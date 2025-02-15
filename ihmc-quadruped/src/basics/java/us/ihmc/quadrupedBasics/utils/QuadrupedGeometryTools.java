package us.ihmc.quadrupedBasics.utils;

import us.ihmc.euclid.referenceFrame.FrameEllipsoid3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.frames.CommonQuadrupedReferenceFrames;

public class QuadrupedGeometryTools
{
   // ONE-OFF METHODS
   
   /**
    * Calculates the leg length of the quadruped.
    * <p>
    * Not real-time.
    * 
    * @param referenceFrames
    * @param robotQuadrant
    * @return legLength
    */
   public static double calculateLegLength(CommonQuadrupedReferenceFrames referenceFrames, RobotQuadrant robotQuadrant)
   {
      ReferenceFrame hipPitchFrame = referenceFrames.getHipPitchFrame(robotQuadrant);
      ReferenceFrame kneePitchFrame = referenceFrames.getKneeFrame(robotQuadrant);
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotQuadrant);

      FramePoint3D hipPitch = new FramePoint3D(hipPitchFrame);
      FramePoint3D kneePitch = new FramePoint3D(kneePitchFrame);
      FramePoint3D foot = new FramePoint3D(footFrame);

      kneePitch.changeFrame(hipPitchFrame);
      double thighLength = kneePitch.distance(hipPitch);

      kneePitch.changeFrame(kneePitchFrame);
      foot.changeFrame(kneePitchFrame);
      double shinLength = foot.distance(kneePitch);

      return thighLength + shinLength;
   }
   
   // REAL-TIME METHODS
   
   /**
    * Updates the feasible step ellipsoid. Assuming projected circles on z=0 for now.
    * <p>
    * Real-time safe, this method keeps track of the available step region for leg of a quadruped robot.
    * 
    * @param referenceFrames
    * @param quadrantHipPitchPoint
    * @param ellipsoidToPackInHipPitchFrame
    * @param robotQuadrant
    */
   public static void updateFootstepWorkspace(RobotQuadrant robotQuadrant, FrameEllipsoid3D ellipsoidToPack, CommonQuadrupedReferenceFrames referenceFrames)
   {
      double hipPitchHeight = ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(referenceFrames.getHipPitchFrame(robotQuadrant)).getM23();
      double maxStepDistance = Math.sqrt(Math.pow(referenceFrames.getLegLength(robotQuadrant), 2) - Math.pow(hipPitchHeight, 2));
      
      ellipsoidToPack.checkReferenceFrameMatch(referenceFrames.getHipPitchFrame(robotQuadrant));
      
      RigidBodyTransform ellipsoidTransform = new RigidBodyTransform();
      ellipsoidTransform.getTranslation().set(0.0, 0.0, -hipPitchHeight);
      ellipsoidToPack.applyTransform(ellipsoidTransform);
      ellipsoidToPack.getRadii().setX(maxStepDistance);
      ellipsoidToPack.getRadii().setY(maxStepDistance);
      ellipsoidToPack.getRadii().setZ(0.0);
   }
}
