package us.ihmc.gdx.ui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiStyleVar;
import imgui.flag.ImGuiWindowFlags;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.GDXImGuiWindowAndDockSystem;
import us.ihmc.gdx.imgui.ImGuiPanelManager;
import us.ihmc.gdx.imgui.ImGuiPanelSizeHandler;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.GDXInputMode;
import us.ihmc.gdx.input.ImGui2DViewInput;
import us.ihmc.gdx.sceneManager.GDX2DSceneManager;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridFile;
import us.ihmc.tools.io.JSONFileTools;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class GDXImGuiBasedUI2D
{
   public static final int ANTI_ALIASING = 2;

   private static final String VIEW_2D_WINDOW_NAME = "2D View";

   private final GDX2DSceneManager sceneManager = new GDX2DSceneManager();
   private final GDXImGuiWindowAndDockSystem imGuiWindowAndDockSystem;
   private final ArrayList<Runnable> onCloseRequestListeners = new ArrayList<>(); // TODO implement on windows closing
   private final String windowTitle;
   private final Path dotIHMCDirectory = Paths.get(System.getProperty("user.home"), ".ihmc");
   private String configurationExtraPath;
   private final HybridDirectory configurationBaseDirectory;
   private HybridFile libGDXSettingsFile;
   private final Stopwatch runTime = new Stopwatch().start();
   private final ImGuiPanelSizeHandler view2DPanelSizeHandler = new ImGuiPanelSizeHandler();
   private ImGui2DViewInput inputCalculator;
   private final ArrayList<Consumer<ImGui2DViewInput>> imgui2DViewInputProcessors = new ArrayList<>();
   private GLFrameBuffer frameBuffer;
   private float sizeX;
   private float sizeY;
   private final ImInt foregroundFPS = new ImInt(240);
   private final ImBoolean vsync = new ImBoolean(false);
   private final ImInt libGDXLogLevel = new ImInt(GDXTools.toGDX(LogTools.getLevel()));
   private final GDXImGuiPerspectiveManager perspectiveManager;

   public GDXImGuiBasedUI2D(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder)
   {
      this(classForLoading, directoryNameToAssumePresent, subsequentPathToResourceFolder, classForLoading.getSimpleName());
   }

   public GDXImGuiBasedUI2D(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder, String windowTitle)
   {
      this.windowTitle = windowTitle;

      configurationExtraPath = "/configurations/" + windowTitle.replaceAll(" ", "");
      configurationBaseDirectory = new HybridDirectory(dotIHMCDirectory,
                                                       directoryNameToAssumePresent,
                                                       subsequentPathToResourceFolder,
                                                       classForLoading,
                                                       configurationExtraPath);

      imGuiWindowAndDockSystem = new GDXImGuiWindowAndDockSystem();
      perspectiveManager = new GDXImGuiPerspectiveManager(classForLoading,
                                                          directoryNameToAssumePresent,
                                                          subsequentPathToResourceFolder,
                                                          configurationExtraPath,
                                                          configurationBaseDirectory);
      perspectiveManager.getPerspectiveDirectoryUpdatedListeners().add(imGuiWindowAndDockSystem::setDirectory);
      perspectiveManager.getPerspectiveDirectoryUpdatedListeners().add(updatedPerspectiveDirectory ->
      {
         libGDXSettingsFile = new HybridFile(updatedPerspectiveDirectory, "GDXSettings.json");
      });
      perspectiveManager.getLoadListeners().add(imGuiWindowAndDockSystem::loadConfiguration);
      perspectiveManager.getLoadListeners().add(loadConfigurationLocation ->
      {
         libGDXSettingsFile.setMode(loadConfigurationLocation.toHybridResourceMode());
         JSONFileTools.load(libGDXSettingsFile.getInputStream(), jsonNode ->
         {
            int width = jsonNode.get("windowWidth").asInt();
            int height = jsonNode.get("windowHeight").asInt();
            Gdx.graphics.setWindowedMode(width, height);
         });
      });
      perspectiveManager.getSaveListeners().add(this::saveApplicationSettings);
      perspectiveManager.applyPerspectiveDirectory();

      imGuiWindowAndDockSystem.getPanelManager().addPrimaryPanel(VIEW_2D_WINDOW_NAME);
   }

   public void launchGDXApplication(Lwjgl3ApplicationAdapter applicationAdapter)
   {
      AtomicReference<Integer> windowWidth = new AtomicReference<>(800);
      AtomicReference<Integer> windowHeight = new AtomicReference<>(600);
      JSONFileTools.loadUserWithClasspathDefaultFallback(libGDXSettingsFile, jsonNode ->
      {
         windowWidth.set(jsonNode.get("windowWidth").asInt());
         windowHeight.set(jsonNode.get("windowHeight").asInt());
      });

      LogTools.info("Launching GDX application");
      GDXApplicationCreator.launchGDXApplication(applicationAdapter, windowTitle, windowWidth.get(), windowHeight.get());
   }

   public void create()
   {
      LogTools.info("Creating...");
      GDXTools.printGLVersion();

      sceneManager.create(GDXInputMode.ImGui);
      inputCalculator = new ImGui2DViewInput(sceneManager.getOrthographicCamera(), this::getViewportSizeX, this::getViewportSizeY);

      Gdx.input.setInputProcessor(null); // detach from getting input events from GDX. TODO: Should we do this here?
      imgui2DViewInputProcessors.add(sceneManager.getOrthographicCamera()::processImGuiInput);



      imGuiWindowAndDockSystem.create(((Lwjgl3Graphics) Gdx.graphics).getWindow().getWindowHandle());

      Runtime.getRuntime().addShutdownHook(new Thread(() -> Gdx.app.exit(), "Exit" + getClass().getSimpleName()));
   }

   public void renderBeforeOnScreenUI()
   {
      Gdx.graphics.setTitle(windowTitle + " - " + Gdx.graphics.getFramesPerSecond() + " FPS");
      imGuiWindowAndDockSystem.beforeWindowManagement();
      render2DView();
      renderMenuBar();
   }

   public void renderEnd()
   {
      imGuiWindowAndDockSystem.afterWindowManagement();
   }

   private void renderMenuBar()
   {
      ImGui.beginMainMenuBar();
      perspectiveManager.renderImGuiPerspectiveMenu();
      if (ImGui.beginMenu("Panels"))
      {
         imGuiWindowAndDockSystem.renderMenuDockPanelItems();
         ImGui.endMenu();
      }
      if (ImGui.beginMenu("Settings"))
      {
         ImGui.pushItemWidth(80.0f);
         if (ImGui.inputInt("Foreground FPS", foregroundFPS, 1))
         {
            Gdx.graphics.setForegroundFPS(foregroundFPS.get());
         }
         if (ImGui.checkbox("Vsync", vsync))
         {
            Gdx.graphics.setVSync(vsync.get());
         }
         if (ImGui.inputInt("libGDX log level", libGDXLogLevel, 1))
         {
            Gdx.app.setLogLevel(libGDXLogLevel.get());
         }
         ImGui.popItemWidth();
         ImGui.endMenu();
      }
      ImGui.sameLine(ImGui.getWindowSizeX() - 170.0f);
      ImGui.text(FormattingTools.getFormattedDecimal2D(runTime.totalElapsed()) + " s");
      ImGui.sameLine(ImGui.getWindowSizeX() - 100.0f);
      ImGui.endMainMenuBar();
   }

   private void render2DView()
   {
      view2DPanelSizeHandler.handleSizeBeforeBegin();
      ImGui.pushStyleVar(ImGuiStyleVar.WindowPadding, 0.0f, 0.0f);
      int flags = ImGuiWindowFlags.None;
      ImGui.begin(VIEW_2D_WINDOW_NAME, flags);
      view2DPanelSizeHandler.handleSizeAfterBegin();

      float posX = ImGui.getWindowPosX();
      float posY = ImGui.getWindowPosY() + ImGuiTools.TAB_BAR_HEIGHT;
      sizeX = ImGui.getWindowSizeX();
      sizeY = ImGui.getWindowSizeY() - ImGuiTools.TAB_BAR_HEIGHT;
      float renderSizeX = sizeX * ANTI_ALIASING;
      float renderSizeY = sizeY * ANTI_ALIASING;

      inputCalculator.compute();
      for (Consumer<ImGui2DViewInput> imGuiInputProcessor : imgui2DViewInputProcessors)
      {
         imGuiInputProcessor.accept(inputCalculator);
      }

      // Allows for dynamically resizing the 2D view panel. Grows by 2x when needed, but never shrinks.
      if (frameBuffer == null || frameBuffer.getWidth() < renderSizeX || frameBuffer.getHeight() < renderSizeY)
      {
         if (frameBuffer != null)
            frameBuffer.dispose();

         int newWidth = frameBuffer == null ? Gdx.graphics.getWidth() * ANTI_ALIASING : frameBuffer.getWidth() * 2;
         int newHeight = frameBuffer == null ? Gdx.graphics.getHeight() * ANTI_ALIASING : frameBuffer.getHeight() * 2;
         LogTools.info("Allocating framebuffer of size: {}x{}", newWidth, newHeight);
         GLFrameBuffer.FrameBufferBuilder frameBufferBuilder = new GLFrameBuffer.FrameBufferBuilder(newWidth, newHeight);
         frameBufferBuilder.addBasicColorTextureAttachment(Pixmap.Format.RGBA8888);
         frameBufferBuilder.addBasicStencilDepthPackedRenderBuffer();
         frameBuffer = frameBufferBuilder.build();
      }

      sceneManager.setViewportBounds((int) renderSizeX, (int) renderSizeY);

      frameBuffer.begin();
      sceneManager.render();
      frameBuffer.end();

      int frameBufferWidth = frameBuffer.getWidth();
      int frameBufferHeight = frameBuffer.getHeight();
      float percentOfFramebufferUsedX = renderSizeX / frameBufferWidth;
      float percentOfFramebufferUsedY = renderSizeY / frameBufferHeight;
      int textureID = frameBuffer.getColorBufferTexture().getTextureObjectHandle();
      float pMinX = posX;
      float pMinY = posY;
      float pMaxX = posX + sizeX;
      float pMaxY = posY + sizeY;
      float uvMinX = 0.0f;
      float uvMinY = percentOfFramebufferUsedY; // flip Y
      float uvMaxX = percentOfFramebufferUsedX;
      float uvMaxY = 0.0f;
      ImGui.getWindowDrawList().addImage(textureID, pMinX, pMinY, pMaxX, pMaxY, uvMinX, uvMinY, uvMaxX, uvMaxY);

      ImGui.end();
      ImGui.popStyleVar();
   }

   private void saveApplicationSettings(ImGuiConfigurationLocation saveConfigurationLocation)
   {
      imGuiWindowAndDockSystem.saveConfiguration(saveConfigurationLocation);
      Consumer<ObjectNode> rootConsumer = root ->
      {
         root.put("windowWidth", Gdx.graphics.getWidth());
         root.put("windowHeight", Gdx.graphics.getHeight());
      };
      if (saveConfigurationLocation == ImGuiConfigurationLocation.VERSION_CONTROL)
      {
         LogTools.info("Saving libGDX settings to {}", libGDXSettingsFile.getWorkspaceFile().toString());
         JSONFileTools.save(libGDXSettingsFile.getWorkspaceFile(), rootConsumer);
      }
      else
      {
         LogTools.info("Saving libGDX settings to {}", libGDXSettingsFile.getExternalFile().toString());
         JSONFileTools.save(libGDXSettingsFile.getExternalFile(), rootConsumer);
      }
   }

   public void dispose()
   {
      imGuiWindowAndDockSystem.dispose();
   }

   public void addOnCloseRequestListener(Runnable onCloseRequest)
   {
      onCloseRequestListeners.add(onCloseRequest);
   }

   public void addImGui2DViewInputProcessor(Consumer<ImGui2DViewInput> processImGuiInput)
   {
      imgui2DViewInputProcessors.add(processImGuiInput);
   }

   public void setVsync(boolean enabled)
   {
      vsync.set(enabled);
      Gdx.graphics.setVSync(enabled);
   }

   public void setForegroundFPS(int foregroundFPS)
   {
      this.foregroundFPS.set(foregroundFPS);
      Gdx.graphics.setForegroundFPS(foregroundFPS);
   }

   public ImGuiPanelManager getImGuiPanelManager()
   {
      return imGuiWindowAndDockSystem.getPanelManager();
   }

   public GDXImGuiPerspectiveManager getPerspectiveManager()
   {
      return perspectiveManager;
   }

   public GDX2DSceneManager get2DSceneManager()
   {
      return sceneManager;
   }

   public float getViewportSizeX()
   {
      return sizeX;
   }

   public float getViewportSizeY()
   {
      return sizeY;
   }

   public GDXImGuiWindowAndDockSystem getImGuiWindowAndDockSystem()
   {
      return imGuiWindowAndDockSystem;
   }

   public HybridDirectory getConfigurationBaseDirectory()
   {
      return configurationBaseDirectory;
   }
}
