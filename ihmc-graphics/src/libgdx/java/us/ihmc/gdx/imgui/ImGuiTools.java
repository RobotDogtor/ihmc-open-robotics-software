package us.ihmc.gdx.imgui;

import com.badlogic.gdx.Input;
import imgui.*;
import imgui.flag.ImGuiFreeTypeBuilderFlags;
import imgui.flag.ImGuiKey;
import org.apache.commons.lang3.SystemUtils;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.geometry.BoundingBox2D;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;

import static org.lwjgl.opengl.GL41.glClear;
import static org.lwjgl.opengl.GL41.glClearColor;

public class ImGuiTools
{
   private static final AtomicInteger GLOBAL_WIDGET_INDEX = new AtomicInteger();
   public static float TAB_BAR_HEIGHT = 20.0f;
   public static final int GDX_TO_IMGUI_KEY_CODE_OFFSET = GLFW.GLFW_KEY_A - Input.Keys.A;
   public static final float FLOAT_MIN = -3.40282346638528859811704183484516925e+38F / 2.0f;
   public static final float FLOAT_MAX = 3.40282346638528859811704183484516925e+38F / 2.0f;

   private static ImFont mediumFont;
   private static ImFont bigFont;
   private static ImFont nodeFont;

   private static boolean userKeysHaveBeenMapped = false;
   private static int spaceKey;
   private static int deleteKey;
   private static int escapeKey;
   private static int upArrowKey;
   private static int downArrowKey;
   private static int leftArrowKey;
   private static int rightArrowKey;

   public static int nextWidgetIndex()
   {
      return GLOBAL_WIDGET_INDEX.getAndIncrement();
   }

   public static void glClearDarkGray()
   {
      glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
      glClear(GL41.GL_COLOR_BUFFER_BIT);
   }

   public static String uniqueLabel(String label)
   {
      return label + "###GlobalWidgetIndex:" + nextWidgetIndex() + ":" + label;
   }

   public static String uniqueLabel(String id, String label)
   {
      return label + "###" + id + ":" + label;
   }

   public static String uniqueLabel(Object thisObject, String label)
   {
      return label + "###" + thisObject.getClass().getName() + ":" + label;
   }

   public static String uniqueIDOnly(Object thisObject, String label)
   {
      return "###" + thisObject.getClass().getName() + ":" + label;
   }

   public static ImFont setupFonts(ImGuiIO io)
   {
      return setupFonts(io, 1);
   }

   /**
    * See https://github.com/ocornut/imgui/blob/master/docs/FONTS.md
    * and ImGuiGlfwFreeTypeDemo in this project
    */
   public static ImFont setupFonts(ImGuiIO io, int fontSizeLevel)
   {
      final ImFontConfig fontConfig = new ImFontConfig(); // Natively allocated object, should be explicitly destroyed
      final ImFontConfig mediumFontConfig = new ImFontConfig();
      final ImFontConfig bigFontConfig = new ImFontConfig();
      final ImFontConfig nodeFontConfig = new ImFontConfig();

      // Glyphs could be added per-font as well as per config used globally like here
//      fontConfig.setGlyphRanges(fontAtlas.getGlyphRangesCyrillic());

//      fontConfig.setMergeMode(true); // When enabled, all fonts added with this config would be merged with the previously added font
      float size = 14.0f;
//      fontConfig.setSizePixels(size);
//      fontConfig.setOversampleH(4);
//      fontConfig.setOversampleV(4);
      int fontsFlags = 0;
      fontsFlags += ImGuiFreeTypeBuilderFlags.LightHinting;
//      fontConfig.setRasterizerFlags(flags);
//      fontConfig.setRasterizerMultiply(2.0f);
//      fontConfig.setPixelSnapH(true);
      fontConfig.setFontBuilderFlags(fontsFlags);
      mediumFontConfig.setFontBuilderFlags(fontsFlags);
      bigFontConfig.setFontBuilderFlags(fontsFlags);
      nodeFontConfig.setFontBuilderFlags(fontsFlags);

      ImFont smallFont;
//      fontToReturn = fontAtlas.addFontDefault(); // Add a default font, which is 'ProggyClean.ttf, 13px'
//      fontToReturn = fontAtlas.addFontFromMemoryTTF(loadFromResources("basis33.ttf"), 16, fontConfig);
      String fontDir;
      if (SystemUtils.IS_OS_WINDOWS) {
         fontDir = System.getenv("WINDIR") + "/Fonts";
      } else {
         fontDir = "/usr/share/fonts/TTF/";
      }

      Path segoeui = Paths.get(fontDir, "segoeui.ttf");
      if (Files.exists(segoeui))
      {
         fontConfig.setName("segoeui.ttf, 16px");
         smallFont = io.getFonts().addFontFromFileTTF(segoeui.toAbsolutePath().toString(), 16.0f, fontConfig);

         fontConfig.setName("segoeui.ttf, 20px");
         mediumFont = io.getFonts().addFontFromFileTTF(segoeui.toAbsolutePath().toString(), 20.0f, mediumFontConfig);

         bigFontConfig.setName("segoeui.ttf, 38px");
         bigFont = io.getFonts().addFontFromFileTTF(segoeui.toAbsolutePath().toString(), 38.0f, bigFontConfig);

         nodeFontConfig.setName("segoeui.ttf, 32px 1/2");
         nodeFont = io.getFonts().addFontFromFileTTF(segoeui.toAbsolutePath().toString(), 32.0f, nodeFontConfig);
      }
      else
      {
         fontConfig.setName("DejaVuSans.ttf, 13px");
         smallFont = io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSans.ttf"), 13.0f, fontConfig);

         fontConfig.setName("DejaVuSans.ttf, 17px");
         mediumFont = io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSans.ttf"), 17.0f, mediumFontConfig);

         bigFontConfig.setName("DejaVuSans.ttf, 32px");
         bigFont = io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSans.ttf"), 32.0f, bigFontConfig);

         nodeFontConfig.setName("DejaVuSans.ttf, 26px 1/2");
         nodeFont = io.getFonts().addFontFromMemoryTTF(ImGuiTools.loadFromResources("dejaVu/DejaVuSans.ttf"), 26.0f, nodeFontConfig);
      }
//      fontConfig.setName("Roboto-Regular.ttf, 14px"); // This name will be displayed in Style Editor
//      fontToReturn = fontAtlas.addFontFromMemoryTTF(loadFromResources("Roboto-Regular.ttf"), size, fontConfig);
//      fontConfig.setName("Roboto-Regular.ttf, 16px"); // We can apply a new config value every time we add a new font
//      fontAtlas.addFontFromMemoryTTF(loadFromResources("Roboto-Regular.ttf"), 16, fontConfig);

//      fontConfig.setName("Segoe UI"); // We can apply a new config value every time we add a new font
//      fontToReturn = fontAtlas.addFontFromFileTTF("/usr/share/fonts/TTF/segoeui.ttf", size, fontConfig);

      nodeFont.setScale(0.5f);

      ImGui.getIO().getFonts().build();

      fontConfig.destroy(); // After all fonts were added we don't need this config more
      mediumFontConfig.destroy();
      bigFontConfig.destroy();
      nodeFontConfig.destroy();

      if (fontSizeLevel == 2)
         return mediumFont;
      if (fontSizeLevel == 3)
         return bigFont;

      return smallFont;
   }

   public static ImFont getBigFont() {
      return bigFont;
   }

   public static ImFont getNodeFont() {
      return nodeFont;
   }

   public static byte[] loadFromResources(final String fileName)
   {
      try (InputStream is = Objects.requireNonNull(ImGuiTools.class.getClassLoader().getResourceAsStream(fileName));
           ByteArrayOutputStream buffer = new ByteArrayOutputStream())
      {

         final byte[] data = new byte[16384];

         int nRead;
         while ((nRead = is.read(data, 0, data.length)) != -1)
         {
            buffer.write(data, 0, nRead);
         }

         return buffer.toByteArray();
      }
      catch (IOException e)
      {
         throw new UncheckedIOException(e);
      }
   }

   public static BoundingBox2D windowBoundingBox()
   {
      BoundingBox2D box = new BoundingBox2D();
      int posX = (int) ImGui.getWindowPosX();
      int posY = (int) ImGui.getWindowPosY();
      box.setMin(posX, posY);
      box.setMax(posX + ImGui.getWindowSizeX(), posY + (int) ImGui.getWindowSizeX());
      return box;
   }

   private static void initializeUserMappedKeys()
   {
      spaceKey = ImGui.getKeyIndex(ImGuiKey.Space);
      deleteKey = ImGui.getKeyIndex(ImGuiKey.Delete);
      escapeKey = ImGui.getKeyIndex(ImGuiKey.Escape);
      upArrowKey = ImGui.getKeyIndex(ImGuiKey.UpArrow);
      downArrowKey = ImGui.getKeyIndex(ImGuiKey.DownArrow);
      leftArrowKey = ImGui.getKeyIndex(ImGuiKey.LeftArrow);
      rightArrowKey = ImGui.getKeyIndex(ImGuiKey.RightArrow);
   }

   public static int getSpaceKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return spaceKey;
   }

   public static int getDeleteKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return deleteKey;
   }

   public static int getEscapeKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return escapeKey;
   }

   public static int getUpArrowKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return upArrowKey;
   }

   public static int getDownArrowKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return downArrowKey;
   }

   public static int getLeftArrowKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return leftArrowKey;
   }

   public static int getRightArrowKey()
   {
      if (!userKeysHaveBeenMapped)
         initializeUserMappedKeys();
      return rightArrowKey;
   }
}
