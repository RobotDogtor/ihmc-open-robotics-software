package us.ihmc.ihmcPerception;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.imageio.ImageIO;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.alg.distort.brown.LensDistortionBrown;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.gui.fiducial.VisualizeFiducial;
import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.calib.CameraPinholeBrown;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageType;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.se.Se3_F64;
import us.ihmc.commons.PrintTools;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class USBFiducialDetector
{
   static
   {
      try
      {
         NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
      }
      catch (UnsatisfiedLinkError e)
      {
         PrintTools.error("Failed to load the OpenCV library.");
      }
   }

   public static void main(String[] args) throws IOException, InterruptedException
   {
      FiducialDetector<GrayF32> detector = FactoryFiducial
            .squareBinary(new ConfigFiducialBinary(0.063), ConfigThreshold.local(ThresholdType.LOCAL_GAUSSIAN, 10), GrayF32.class);

      VideoCapture videoCapture = new VideoCapture(0);
      // You need to run camera calibration http://boofcv.org/index.php?title=Tutorial_Camera_Calibration
      // unless you are sure you are using the correct intrinsic parameters
      // Not sure what these were for
      //      CameraPinholeBrown intrinsicParameters = new CameraPinholeBrown(476, 476, 0, 640, 360, 1280, 720);
      // These are for the Logitech USB camera on the 7bot table
      CameraPinholeBrown intrinsicParameters = new CameraPinholeBrown(801.3016379311194, 800.9022148463515, 0.0, 333.13411944271314, 231.9954112461063, 640,
            480);
      detector.setLensDistortion(new LensDistortionBrown(intrinsicParameters), intrinsicParameters.width, intrinsicParameters.height);

      ImagePanel imagePanel = null;
      Mat image = new Mat();
      MatOfByte matOfByte = new MatOfByte();
      GrayF32 formattedImage;
      double[] eulerAngles = new double[3];

      while (true)
      {
         Thread.sleep(10);
         try
         {
            videoCapture.read(image);

            Imgcodecs.imencode(".bmp", image, matOfByte);
            BufferedImage bufferedImage = ImageIO.read(new ByteArrayInputStream(matOfByte.toArray()));
            //            System.out.println("bufferedImage = " + bufferedImage.getWidth() + ":" + bufferedImage.getHeight());

            formattedImage = ConvertBufferedImage.convertFrom(bufferedImage, true, ImageType.single(GrayF32.class));

            detector.detect(formattedImage);

            Graphics2D g2 = bufferedImage.createGraphics();
            Se3_F64 fiducialToCamera = new Se3_F64();
            NumberFormat numberFormat = new DecimalFormat("0.00");
            for (int i = 0; i < detector.totalFound(); i++)
            {
               detector.getFiducialToCamera(i, fiducialToCamera);

               VisualizeFiducial.drawCube(fiducialToCamera, intrinsicParameters, detector.getWidth(i), 3, g2);
               VisualizeFiducial.drawLabelCenter(fiducialToCamera, intrinsicParameters, "" + detector.getId(i), g2);

               long id = detector.getId(i);
               ConvertRotation3D_F64.matrixToEuler(fiducialToCamera.R, EulerType.XYZ, eulerAngles);
               System.out.println(
                     "target " + id + ": (" + numberFormat.format(fiducialToCamera.getX()) + ", " + numberFormat.format(fiducialToCamera.getY()) + ", "
                           + numberFormat.format(fiducialToCamera.getZ()) + ") (" + numberFormat.format(Math.toDegrees(eulerAngles[0])) + ", " + numberFormat
                           .format(Math.toDegrees(eulerAngles[1])) + ", " + numberFormat.format(Math.toDegrees(eulerAngles[2])) + ")");
            }

            if (imagePanel == null)
            {
               imagePanel = ShowImages.showWindow(bufferedImage, "video");
            }
            else
            {
               imagePanel.setImageUI(bufferedImage);
            }
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      }
   }
}
