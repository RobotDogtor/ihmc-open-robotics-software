package us.ihmc.gdx.simulation.sensors;

import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.glutils.*;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import org.apache.commons.lang3.tuple.Pair;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.gdx.perception.GDXBytedecoImage;
import us.ihmc.gdx.perception.GDXCVImagePanel;
import us.ihmc.gdx.sceneManager.GDX3DSceneBasics;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.DepthSensorShaderProvider;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.visualizers.GDXFrustumVisualizer;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.Random;

/**
 * Lidar could be simulated with a viewportHeight of 1 and rotating the camera
 */
public class GDXLowLevelDepthSensorSimulator
{
   private final String depthWindowName;
   private final String colorWindowName;

   private final Random random = new Random();

   private final int imageWidth;
   private final int imageHeight;
   private final int numberOfPoints;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImFloat fieldOfViewY = new ImFloat();
   private final ImFloat focalLengthPixels = new ImFloat();
   private final ImFloat nearPlaneDistance = new ImFloat();
   private final ImFloat farPlaneDistance = new ImFloat();
   private final ImFloat principalOffsetXPixels = new ImFloat();
   private final ImFloat principalOffsetYPixels = new ImFloat();
   private final double updatePeriod;
   private final Timer throttleTimer = new Timer();
   private final Vector3D32 noiseVector = new Vector3D32();

   /** Simulated camera that observes the current GDX Scene **/
   private PerspectiveCamera camera;
   private RigidBodyTransform transformToWorldFrame = new RigidBodyTransform();
   private ModelBatch modelBatch;
   private ScreenViewport viewport;
   private SensorFrameBuffer frameBuffer;
   private boolean depthEnabled = true;
   private final ImBoolean renderFrustum = new ImBoolean(false);

   private GDXCVImagePanel depthPanel;
   private ImGuiVideoPanel colorPanel;
   private GDXFrustumVisualizer frustumVisualizer;

   private OpenCLManager openCLManager;
   private _cl_kernel openCLKernel;
   private GDXBytedecoImage normalizedDeviceCoordinateDepthImage;
   // https://stackoverflow.com/questions/14435632/impulse-gaussian-and-salt-and-pepper-noise-with-opencv
   private Mat noiseLow;
   private Mat noiseHigh;
   /** This generates a random matrix of uniformly distributed depth noise according to a random distribution **/
   private GDXBytedecoImage noiseImage; // TODO: Salt and pepper?
   /** This contains a row major float buffer that has the depth of each pixel in the image **/
   private GDXBytedecoImage metersDepthImage;
   /** This contains an int buffer that has a color for each point contains in {pointCloudRenderingBuffer} **/
   private GDXBytedecoImage rgba8888ColorImage;
   /** This contains a float buffer that has the point cloud in world coordinates **/
   private OpenCLFloatBuffer pointCloudRenderingBuffer;
   private OpenCLFloatBuffer parametersBuffer;
   private boolean firstRender = true;

   public GDXLowLevelDepthSensorSimulator(String sensorName, double fieldOfViewY, int imageWidth, int imageHeight, double minRange, double maxRange)
   {
      depthWindowName = ImGuiTools.uniqueLabel(sensorName + " Depth");
      colorWindowName = ImGuiTools.uniqueLabel(sensorName + " Color");
      this.fieldOfViewY.set((float) fieldOfViewY);
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      numberOfPoints = imageWidth * imageHeight;
      principalOffsetXPixels.set(imageWidth / 2.0f);
      principalOffsetYPixels.set(imageHeight / 2.0f);
      nearPlaneDistance.set((float) minRange);
      farPlaneDistance.set((float) maxRange);
      calculateFocalLength();
      this.updatePeriod = UnitConversions.hertzToSeconds(30.0);
   }

   public void create()
   {
      create(null);
   }

   public void create(FloatBuffer pointCloudRenderingBufferToPack)
   {
      throttleTimer.reset();

      camera = new PerspectiveCamera(fieldOfViewY.get(), imageWidth, imageHeight);
      camera.near = nearPlaneDistance.get();
      camera.far = farPlaneDistance.get();
      viewport = new ScreenViewport(camera);

      Pair<String, String> shaderStrings = GDXTools.loadCombinedShader(getClass().getName().replace(".", "/") + ".glsl");
      String vertexShader = shaderStrings.getLeft();
      String fragmentShader = shaderStrings.getRight();
      modelBatch = new ModelBatch(null, new DepthSensorShaderProvider(vertexShader, fragmentShader), null);

      SensorFrameBufferBuilder frameBufferBuilder = new SensorFrameBufferBuilder(imageWidth, imageHeight);
      frameBufferBuilder.addColorTextureAttachment(GL41.GL_RGBA8, GL41.GL_RGBA, GL41.GL_UNSIGNED_BYTE);
      frameBufferBuilder.addDepthTextureAttachment(GL41.GL_DEPTH_COMPONENT32F, GL41.GL_FLOAT);
      frameBufferBuilder.addColorTextureAttachment(GL41.GL_R32F, GL41.GL_RED, GL41.GL_FLOAT);
      frameBuffer = frameBufferBuilder.build();

      openCLManager = new OpenCLManager();
      openCLManager.create();
      openCLKernel = openCLManager.loadSingleFunctionProgramAndCreateKernel("LowLevelDepthSensorSimulator");

      normalizedDeviceCoordinateDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1);
      noiseImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1);
      noiseLow = new Mat(1, 1, opencv_core.CV_32FC1);
      noiseLow.ptr().putFloat(0.0035f);
      noiseHigh = new Mat(1, 1, opencv_core.CV_32FC1);
      noiseHigh.ptr().putFloat(-0.0035f);
      metersDepthImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1);
      rgba8888ColorImage = new GDXBytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4);
      if (pointCloudRenderingBufferToPack != null)
         pointCloudRenderingBuffer = new OpenCLFloatBuffer(numberOfPoints * 8, pointCloudRenderingBufferToPack);
      else
         pointCloudRenderingBuffer = new OpenCLFloatBuffer(1);
      parametersBuffer = new OpenCLFloatBuffer(28);

      // TODO these panels should be removable to a separate class
      depthPanel = new GDXCVImagePanel(depthWindowName, imageWidth, imageHeight);
      colorPanel = new ImGuiVideoPanel(colorWindowName, true);
      colorPanel.setTexture(frameBuffer.getColorTexture());

      frustumVisualizer = new GDXFrustumVisualizer();
   }

   private void calculateFocalLength()
   {
      focalLengthPixels.set((float) ((imageHeight / 2.0) / Math.tan(Math.toRadians((fieldOfViewY.get() / 2.0)))));
   }

   private void calculateFieldOfView()
   {
      fieldOfViewY.set(2.0f * (float) Math.toDegrees(Math.atan((imageHeight / 2.0) / focalLengthPixels.get())));
   }

   public void render(GDX3DSceneManager sceneManager)
   {
      render(sceneManager, null, 0.01f);
   }

   public void render(GDX3DSceneBasics sceneBasics)
   {
      render(sceneBasics, null, 0.01f);
   }

   public void render(GDX3DSceneManager sceneManager, Color userPointColor, float pointSize)
   {
      render(sceneManager.getSceneBasics(), userPointColor, pointSize);
   }

   public void render(GDX3DSceneBasics sceneBasics, Color userPointColor, float pointSize)
   {
      boolean updateThisTick = throttleTimer.isExpired(updatePeriod);
      if (updateThisTick)
         throttleTimer.reset();
      else
         return;

      frameBuffer.begin();

      float clear = 0.0f;
      GL41.glClearColor(clear, clear, clear, 1.0f);
      GL41.glClear(GL41.GL_COLOR_BUFFER_BIT | GL41.GL_DEPTH_BUFFER_BIT);

      viewport.update(imageWidth, imageHeight);
      if (renderFrustum.get())
      {
         frustumVisualizer.generateMesh(camera.frustum);
         frustumVisualizer.update();
      }
      modelBatch.begin(camera);
      GL41.glViewport(0, 0, imageWidth, imageHeight);

      sceneBasics.renderExternalBatch(modelBatch, GDXSceneLevel.REAL_ENVIRONMENT);

      modelBatch.end();

      GL41.glReadBuffer(GL41.GL_COLOR_ATTACHMENT0);
      GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 4); // to read ints
      rgba8888ColorImage.getBackingDirectByteBuffer().rewind();
      GL41.glReadPixels(0, 0, imageWidth, imageHeight, GL41.GL_RGBA, GL41.GL_UNSIGNED_INT_8_8_8_8, rgba8888ColorImage.getBackingDirectByteBuffer());
      GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 1); // undo what we did

      if (depthEnabled)
      {
         normalizedDeviceCoordinateDepthImage.getBackingDirectByteBuffer().rewind(); // SIGSEV otherwise
         GL41.glReadBuffer(GL41.GL_COLOR_ATTACHMENT1);
         GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 4); // to read floats
         GL41.glReadPixels(0, 0, imageWidth, imageHeight, GL41.GL_RED, GL41.GL_FLOAT, normalizedDeviceCoordinateDepthImage.getBackingDirectByteBuffer());
         GL41.glPixelStorei(GL41.GL_UNPACK_ALIGNMENT, 1); // undo what we did
      }

      frameBuffer.end();

      opencv_core.randu(noiseImage.getBytedecoOpenCVMat(), noiseLow, noiseHigh);

      parametersBuffer.getBytedecoFloatBufferPointer().put(0, camera.near);
      parametersBuffer.getBytedecoFloatBufferPointer().put(1, camera.far);
      parametersBuffer.getBytedecoFloatBufferPointer().put(2, principalOffsetXPixels.get());
      parametersBuffer.getBytedecoFloatBufferPointer().put(3, principalOffsetYPixels.get());
      parametersBuffer.getBytedecoFloatBufferPointer().put(4, focalLengthPixels.get());
      float calculatePointCloud = pointCloudRenderingBuffer.getNumberOfFloats() > 1 ? 1.0f : 0.0f;
      parametersBuffer.getBytedecoFloatBufferPointer().put(5, calculatePointCloud);
      parametersBuffer.getBytedecoFloatBufferPointer().put(6, imageWidth);
      parametersBuffer.getBytedecoFloatBufferPointer().put(7, imageHeight);
      parametersBuffer.getBytedecoFloatBufferPointer().put(8, pointSize);
      parametersBuffer.getBytedecoFloatBufferPointer().put(9, userPointColor == null ? -1.0f : userPointColor.r);
      parametersBuffer.getBytedecoFloatBufferPointer().put(10, userPointColor == null ? -1.0f : userPointColor.g);
      parametersBuffer.getBytedecoFloatBufferPointer().put(11, userPointColor == null ? -1.0f : userPointColor.b);
      parametersBuffer.getBytedecoFloatBufferPointer().put(12, userPointColor == null ? -1.0f : userPointColor.a);
      parametersBuffer.getBytedecoFloatBufferPointer().put(13, transformToWorldFrame.getTranslation().getX32());
      parametersBuffer.getBytedecoFloatBufferPointer().put(14, transformToWorldFrame.getTranslation().getY32());
      parametersBuffer.getBytedecoFloatBufferPointer().put(15, transformToWorldFrame.getTranslation().getZ32());
      parametersBuffer.getBytedecoFloatBufferPointer().put(16, (float) transformToWorldFrame.getRotation().getM00());
      parametersBuffer.getBytedecoFloatBufferPointer().put(17, (float) transformToWorldFrame.getRotation().getM01());
      parametersBuffer.getBytedecoFloatBufferPointer().put(18, (float) transformToWorldFrame.getRotation().getM02());
      parametersBuffer.getBytedecoFloatBufferPointer().put(19, (float) transformToWorldFrame.getRotation().getM10());
      parametersBuffer.getBytedecoFloatBufferPointer().put(20, (float) transformToWorldFrame.getRotation().getM11());
      parametersBuffer.getBytedecoFloatBufferPointer().put(21, (float) transformToWorldFrame.getRotation().getM12());
      parametersBuffer.getBytedecoFloatBufferPointer().put(22, (float) transformToWorldFrame.getRotation().getM20());
      parametersBuffer.getBytedecoFloatBufferPointer().put(23, (float) transformToWorldFrame.getRotation().getM21());
      parametersBuffer.getBytedecoFloatBufferPointer().put(24, (float) transformToWorldFrame.getRotation().getM22());
      parametersBuffer.getBytedecoFloatBufferPointer().put(25, camera.position.x);
      parametersBuffer.getBytedecoFloatBufferPointer().put(26, camera.position.y);
      parametersBuffer.getBytedecoFloatBufferPointer().put(27, camera.position.z);
      if (firstRender)
      {
         firstRender = false;
         normalizedDeviceCoordinateDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
         noiseImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
         rgba8888ColorImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
         metersDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);
         pointCloudRenderingBuffer.createOpenCLBufferObject(openCLManager);
         parametersBuffer.createOpenCLBufferObject(openCLManager);
      }
      else
      {
         normalizedDeviceCoordinateDepthImage.writeOpenCLImage(openCLManager);
         noiseImage.writeOpenCLImage(openCLManager);
         rgba8888ColorImage.writeOpenCLImage(openCLManager);
         parametersBuffer.writeOpenCLBufferObject(openCLManager);
      }
      openCLManager.setKernelArgument(openCLKernel, 0, normalizedDeviceCoordinateDepthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(openCLKernel, 1, noiseImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(openCLKernel, 2, rgba8888ColorImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(openCLKernel, 3, metersDepthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(openCLKernel, 4, pointCloudRenderingBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(openCLKernel, 5, parametersBuffer.getOpenCLBufferObject());
      openCLManager.execute2D(openCLKernel, imageWidth, imageHeight);
      metersDepthImage.readOpenCLImage(openCLManager);
      pointCloudRenderingBuffer.readOpenCLBufferObject(openCLManager);
      openCLManager.finish();

      if (depthPanel.getVideoPanel().getIsShowing().get())
         depthPanel.drawFloatImage(metersDepthImage.getBytedecoOpenCVMat());
   }

   public void renderTuningSliders()
   {
      if (ImGui.sliderFloat(labels.get("Field of view Y (deg)"), fieldOfViewY.getData(), 1.0f, 180.0f))
      {
         camera.fieldOfView = fieldOfViewY.get();
         calculateFocalLength();
      }
      if (ImGui.sliderFloat(labels.get("Near plane distance (m)"), nearPlaneDistance.getData(), 0.01f, 0.2f))
         camera.near = nearPlaneDistance.get();
      if (ImGui.sliderFloat(labels.get("Far plane distance (m)"), farPlaneDistance.getData(), 0.21f, 5.0f))
         camera.far = farPlaneDistance.get();
      if (ImGui.sliderFloat(labels.get("Focal length (px)"), focalLengthPixels.getData(), -1000.0f, 1000.0f))
      {
         calculateFieldOfView();
         camera.fieldOfView = fieldOfViewY.get();
      }
      ImGui.checkbox(labels.get("Render frustum"), renderFrustum);
      ImGui.sliderFloat(labels.get("Principal Offset X (px)"), principalOffsetXPixels.getData(), -imageWidth, imageWidth);
      ImGui.sliderFloat(labels.get("Principal Offset Y (px)"), principalOffsetYPixels.getData(), -imageHeight, imageHeight);
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderFrustum.get())
         frustumVisualizer.getRenderables(renderables, pool);
   }

   public void dispose()
   {
      frameBuffer.dispose();
      modelBatch.dispose();
      // TODO: There's a lot more to dispose here, probably
      openCLManager.destroy();
   }

   public void setCameraWorldTransform(Matrix4 worldTransform)
   {
      camera.position.setZero();
      camera.up.set(0.0f, 0.0f, 1.0f);
      camera.direction.set(1.0f, 0.0f, 0.0f);
      camera.transform(worldTransform);
      GDXTools.toEuclid(worldTransform, transformToWorldFrame);
   }

   public PerspectiveCamera getCamera()
   {
      return camera;
   }

   public ByteBuffer getMetersDepthFloatBuffer()
   {
      return metersDepthImage.getBackingDirectByteBuffer();
   }

   /**
    * OpenGL and OpenCV have different orderings, this data is ABGR, but OpenGl reads
    * each pixel right to left so to OpenGL it's RGBA.
    */
   public GDXBytedecoImage getABGR8888ColorImage()
   {
      return rgba8888ColorImage;
   }

   public ByteBuffer getColorRGBA8Buffer()
   {
      return rgba8888ColorImage.getBackingDirectByteBuffer();
   }

   /**
    * Returns the actual point cloud
    */
   public FloatBuffer getPointCloudBuffer()
   {
      return pointCloudRenderingBuffer.getBackingDirectFloatBuffer();
   }

   public float getMaxRange()
   {
      return farPlaneDistance.get();
   }

   public ImGuiVideoPanel getDepthPanel()
   {
      return depthPanel.getVideoPanel();
   }

   public ImGuiVideoPanel getColorPanel()
   {
      return colorPanel;
   }

   public void setDepthEnabled(boolean depthEnabled)
   {
      this.depthEnabled = depthEnabled;
   }

   public ImFloat getPrincipalOffsetXPixels()
   {
      return principalOffsetXPixels;
   }

   public ImFloat getPrincipalOffsetYPixels()
   {
      return principalOffsetYPixels;
   }

   public ImFloat getFocalLengthPixels()
   {
      return focalLengthPixels;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public int getNumberOfPoints()
   {
      return numberOfPoints;
   }
}
