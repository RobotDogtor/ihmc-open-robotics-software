package us.ihmc.humanoidRobotics.frames;

import java.util.EnumMap;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.model.CenterOfMassStateProvider;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingMidFootZUpGroundFrame;
import us.ihmc.robotics.screwTheory.MovingMidFrameZUpFrame;
import us.ihmc.robotics.screwTheory.MovingZUpFrame;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.tools.containers.ContainerTools;

public class HumanoidReferenceFrames implements CommonHumanoidReferenceFrames
{
   private final FullHumanoidRobotModel fullRobotModel;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final TLongObjectHashMap<ReferenceFrame> nameBasedHashCodeToReferenceFrameMap = new TLongObjectHashMap<ReferenceFrame>();

   private final MovingReferenceFrame chestFrame;
   private final MovingReferenceFrame pelvisFrame;
   private final MovingZUpFrame pelvisZUpFrame;

   private final EnumMap<SpineJointName, MovingReferenceFrame> spineReferenceFrames = ContainerTools.createEnumMap(SpineJointName.class);
   private final EnumMap<NeckJointName, MovingReferenceFrame> neckReferenceFrames = ContainerTools.createEnumMap(NeckJointName.class);
   private final SideDependentList<EnumMap<ArmJointName, MovingReferenceFrame>> armJointFrames = SideDependentList.createListOfEnumMaps(ArmJointName.class);
   private final SideDependentList<EnumMap<LegJointName, MovingReferenceFrame>> legJointFrames = SideDependentList.createListOfEnumMaps(LegJointName.class);

   private final SideDependentList<MovingReferenceFrame> handZUpFrames = new SideDependentList<>();
   private final SideDependentList<MovingReferenceFrame> ankleZUpFrames = new SideDependentList<>();
   private final SideDependentList<MovingReferenceFrame> footReferenceFrames = new SideDependentList<>();
   private final SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<MovingReferenceFrame> soleZUpFrames = new SideDependentList<>();
   private final MovingMidFrameZUpFrame midFeetZUpFrame;
   private final MovingMidFootZUpGroundFrame midFootZUpGroundFrame;
   private final MovingReferenceFrame midFeetUnderPelvisWalkDirectionFrame;

   private final ReferenceFrame centerOfMassFrame;
   private final ReferenceFrame lidarSensorFrame;
   private ReferenceFrame headCameraFrame;
   private ReferenceFrame steppingCameraFrame;
   private ReferenceFrame objectDetectionCameraFrame;
   private ReferenceFrame headZED2CameraFrame;
   private ReferenceFrame ousterLidarFrame;

   public HumanoidReferenceFrames(FullHumanoidRobotModel fullRobotModel)
   {
      this(fullRobotModel, null);
   }

   public HumanoidReferenceFrames(FullHumanoidRobotModel fullRobotModel, HumanoidRobotSensorInformation sensorInformation)
   {
      this(fullRobotModel, new CenterOfMassReferenceFrame("centerOfMass", worldFrame, fullRobotModel.getElevator()), sensorInformation);
   }

   public HumanoidReferenceFrames(FullHumanoidRobotModel fullRobotModel,
                                  CenterOfMassStateProvider centerOfMassStateProvider,
                                  HumanoidRobotSensorInformation sensorInformation)
   {
      this(fullRobotModel, new ReferenceFrame("centerOfMassFrame", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.getTranslation().set(centerOfMassStateProvider.getCenterOfMassPosition());
         }
      }, sensorInformation);
   }

   public HumanoidReferenceFrames(FullHumanoidRobotModel fullRobotModel, ReferenceFrame centerOfMassFrame, HumanoidRobotSensorInformation sensorInformation)
   {
      this.fullRobotModel = fullRobotModel;
      this.centerOfMassFrame = centerOfMassFrame;

      if (fullRobotModel.getPelvis() != null)
      {
         pelvisFrame = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint();
      }
      else
      {
         pelvisFrame = null;
      }
      if (fullRobotModel.getChest() != null)
      {
         chestFrame = fullRobotModel.getChest().getParentJoint().getFrameAfterJoint();
      }
      else
      {
         chestFrame = null;
      }
      pelvisZUpFrame = new MovingZUpFrame(pelvisFrame, "pelvisZUpFrame");

      RobotSpecificJointNames robotJointNames = fullRobotModel.getRobotSpecificJointNames();

      if (robotJointNames.getNeckJointNames() != null)
      {
         for (NeckJointName neckJointName : robotJointNames.getNeckJointNames())
         {
            this.neckReferenceFrames.put(neckJointName, fullRobotModel.getNeckJoint(neckJointName).getFrameAfterJoint());
         }
      }

      if (!fullRobotModel.getLidarSensorNames().isEmpty() && fullRobotModel.getLidarSensorNames().get(0) != null
            && !fullRobotModel.getLidarSensorNames().get(0).isEmpty())
      {
         String lidarSensorName = fullRobotModel.getLidarSensorNames().get(0);
         ReferenceFrame lidarBaseFrame = fullRobotModel.getLidarBaseFrame(lidarSensorName);
         RigidBodyTransform lidarBaseToSensorTransform = fullRobotModel.getLidarBaseToSensorTransform(lidarSensorName);
         if (lidarBaseFrame != null && lidarBaseToSensorTransform != null)
         {
            lidarSensorFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("lidarSensorFrame",
                                                                                                 lidarBaseFrame,
                                                                                                 lidarBaseToSensorTransform);
         }
         else
         {
            lidarSensorFrame = null;
         }
      }
      else
      {
         lidarSensorFrame = null;
      }
      if (sensorInformation != null)
      {
         headCameraFrame = fullRobotModel.getCameraFrame(sensorInformation.getHeadCameraName());
      }

      if (robotJointNames.getSpineJointNames() != null)
      {
         for (SpineJointName spineJointName : robotJointNames.getSpineJointNames())
         {
            this.spineReferenceFrames.put(spineJointName, fullRobotModel.getSpineJoint(spineJointName).getFrameAfterJoint());
         }
      }

      if (robotJointNames.getArmJointNames() != null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            for (ArmJointName armJointName : robotJointNames.getArmJointNames())
            {
               this.armJointFrames.get(robotSide).put(armJointName, fullRobotModel.getArmJoint(robotSide, armJointName).getFrameAfterJoint());
            }
         }
      }

      if (robotJointNames.getLegJointNames() != null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            for (LegJointName legJointName : robotJointNames.getLegJointNames())
            {
               MovingReferenceFrame legJointFrame = fullRobotModel.getFrameAfterLegJoint(robotSide, legJointName);
               legJointFrames.get(robotSide).put(legJointName, legJointFrame);
            }
         }
      }

      SideDependentList<MovingZUpFrame> localSoleZUpFrames = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         MovingReferenceFrame footFrame = getFootFrame(robotSide);
         footReferenceFrames.put(robotSide, footFrame);

         MovingZUpFrame ankleZUpFrame = new MovingZUpFrame(footFrame, robotSide.getCamelCaseNameForStartOfExpression() + "AnkleZUp");
         ankleZUpFrames.put(robotSide, ankleZUpFrame);

         MovingReferenceFrame handFrame = getHandFrame(robotSide);
         if (handFrame != null)
         {
            MovingZUpFrame handZUpFrame = new MovingZUpFrame(handFrame, robotSide.getCamelCaseNameForStartOfExpression() + "HandZUp");
            handZUpFrames.put(robotSide, handZUpFrame);
         }
         MovingReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
         soleFrames.put(robotSide, soleFrame);

         MovingZUpFrame soleZUpFrame = new MovingZUpFrame(soleFrame, soleFrame.getName() + "ZUp");
         localSoleZUpFrames.put(robotSide, soleZUpFrame);
         soleZUpFrames.put(robotSide, soleZUpFrame);
      }

      midFeetZUpFrame = new MovingMidFrameZUpFrame("midFeetZUp", getSoleFrame(RobotSide.LEFT), getSoleFrame(RobotSide.RIGHT));
      midFootZUpGroundFrame = new MovingMidFootZUpGroundFrame("midFeetZUpAverageYaw",
                                                              localSoleZUpFrames.get(RobotSide.LEFT),
                                                              localSoleZUpFrames.get(RobotSide.RIGHT));

      midFeetUnderPelvisWalkDirectionFrame = new MovingWalkingReferenceFrame("walkingFrame", pelvisFrame, midFootZUpGroundFrame);

      // set default CommonHumanoidReferenceFrameIds for certain frames used commonly for control
      addDefaultIDToReferenceFrame(CommonReferenceFrameIds.MIDFEET_ZUP_FRAME, getMidFeetZUpFrame());
      addDefaultIDToReferenceFrame(CommonReferenceFrameIds.PELVIS_ZUP_FRAME, getPelvisZUpFrame());
      addDefaultIDToReferenceFrame(CommonReferenceFrameIds.PELVIS_FRAME, getPelvisFrame());
      addDefaultIDToReferenceFrame(CommonReferenceFrameIds.CENTER_OF_MASS_FRAME, getCenterOfMassFrame());
      addDefaultIDToReferenceFrame(CommonReferenceFrameIds.LEFT_SOLE_FRAME, getSoleFrame(RobotSide.LEFT));
      addDefaultIDToReferenceFrame(CommonReferenceFrameIds.RIGHT_SOLE_FRAME, getSoleFrame(RobotSide.RIGHT));
      RigidBodyBasics chest = fullRobotModel.getChest();
      if (chest != null)
      {
         addDefaultIDToReferenceFrame(CommonReferenceFrameIds.CHEST_FRAME, chest.getBodyFixedFrame());
      }

      if (sensorInformation != null)
      {
         steppingCameraFrame = sensorInformation.getSteppingCameraFrame(this);
         objectDetectionCameraFrame = sensorInformation.getObjectDetectionCameraFrame(this);
         headZED2CameraFrame = sensorInformation.getHeadZED2CameraFrame(this);
         ousterLidarFrame = sensorInformation.getOusterLidarFrame(this);
      }
   }

   private void addDefaultIDToReferenceFrame(CommonReferenceFrameIds commonId, ReferenceFrame referenceFrame)
   {
      referenceFrame.setAdditionalNameBasedHashCode(commonId.getHashId());
      nameBasedHashCodeToReferenceFrameMap.put(commonId.getHashId(), referenceFrame);
   }

   @Override
   public SideDependentList<MovingReferenceFrame> getAnkleZUpReferenceFrames()
   {
      return ankleZUpFrames;
   }

   @Override
   public SideDependentList<MovingReferenceFrame> getFootReferenceFrames()
   {
      return footReferenceFrames;
   }

   @Override
   public MovingReferenceFrame getFootFrame(RobotSide robotSide)
   {
      return fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
   }

   public static ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   // TORSO
   public MovingReferenceFrame getSpineFrame(SpineJointName spineJointName)
   {
      return spineReferenceFrames.get(spineJointName);
   }

   @Override
   public MovingReferenceFrame getABodyAttachedZUpFrame()
   {
      return pelvisZUpFrame;
   }

   /**
    * Return the ReferenceFrame located after the parent joint of the pelvis.
    */
   @Override
   public MovingReferenceFrame getPelvisZUpFrame()
   {
      return pelvisZUpFrame;
   }

   /**
    * Return the ReferenceFrame located after the parent joint of the chest.
    */
   @Override
   public MovingReferenceFrame getChestFrame()
   {
      return chestFrame;
   }

   @Override
   public MovingReferenceFrame getPelvisFrame()
   {
      return pelvisFrame;
   }

   public MovingReferenceFrame getNeckFrame(NeckJointName neckJointName)
   {
      return neckReferenceFrames.get(neckJointName);
   }

   public MovingReferenceFrame getArmFrame(RobotSide robotSide, ArmJointName armJointName)
   {
      return armJointFrames.get(robotSide).get(armJointName);
   }

   public MovingReferenceFrame getHandZUpFrame(RobotSide robotSide)
   {
      return handZUpFrames.get(robotSide);
   }

   // LEGS
   @Override
   public MovingReferenceFrame getLegJointFrame(RobotSide robotSide, LegJointName legJointName)
   {
      return legJointFrames.get(robotSide).get(legJointName);
   }

   @Override
   public MovingReferenceFrame getAnkleZUpFrame(RobotSide robotSide)
   {
      return ankleZUpFrames.get(robotSide);
   }

   @Override
   public MovingReferenceFrame getMidFeetZUpFrame()
   {
      return midFeetZUpFrame;
   }

   @Override
   public MovingReferenceFrame getMidFootZUpGroundFrame()
   {
      return midFootZUpGroundFrame;
   }

   /**
    * Return the ReferenceFrame located between the feet (to remove robot swaying left to right) but
    * under the pelvis (to remove forward and backwards swaying)
    */
   @Override
   public MovingReferenceFrame getMidFeetUnderPelvisFrame()
   {
      return midFeetUnderPelvisWalkDirectionFrame;
   }

   /**
    * Returns the control frame of a hand. The control frame is robot specific and is meant to be
    * located at a key position for grasping.
    * <p>
    * TODO rename the method to getHandControlFrame(RobotSide).
    * 
    * @param robotSide from which hand the control frame has to be returned.
    * @return the hand control frame.
    */
   public MovingReferenceFrame getHandFrame(RobotSide robotSide)
   {
      return fullRobotModel.getHandControlFrame(robotSide);
   }

   @Override
   public void updateFrames()
   {
      fullRobotModel.updateFrames();

      pelvisZUpFrame.update();

      for (RobotSide robotSide : RobotSide.values)
      {
         ankleZUpFrames.get(robotSide).update();

         ReferenceFrame handZUpFrame = handZUpFrames.get(robotSide);
         if (handZUpFrame != null)
         {
            handZUpFrame.update();
         }
         footReferenceFrames.get(robotSide).update();
         soleFrames.get(robotSide).update();
         soleZUpFrames.get(robotSide).update();
      }

      midFeetZUpFrame.update();
      midFootZUpGroundFrame.update();
      midFeetUnderPelvisWalkDirectionFrame.update();

      centerOfMassFrame.update();

      if (lidarSensorFrame != null)
         lidarSensorFrame.update();
      if (steppingCameraFrame != null)
      {
         steppingCameraFrame.update();
         objectDetectionCameraFrame.update();
         headZED2CameraFrame.update();
         ousterLidarFrame.update();
      }
   }

   @Override
   public EnumMap<LegJointName, MovingReferenceFrame> getLegJointFrames(RobotSide robotSide)
   {
      return legJointFrames.get(robotSide);
   }

   @Override
   public ReferenceFrame getIMUFrame()
   {
      return pelvisFrame;
   }

   @Override
   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   @Override
   public MovingReferenceFrame getSoleFrame(RobotSide robotSide)
   {
      return soleFrames.get(robotSide);
   }

   @Override
   public SideDependentList<MovingReferenceFrame> getSoleFrames()
   {
      return soleFrames;
   }

   @Override
   public MovingReferenceFrame getSoleZUpFrame(RobotSide robotSide)
   {
      return soleZUpFrames.get(robotSide);
   }

   @Override
   public SideDependentList<MovingReferenceFrame> getSoleZUpFrames()
   {
      return soleZUpFrames;
   }

   @Override
   public TLongObjectHashMap<ReferenceFrame> getReferenceFrameDefaultHashIds()
   {
      return nameBasedHashCodeToReferenceFrameMap;
   }

   public ReferenceFrame getLidarSensorFrame()
   {
      return lidarSensorFrame;
   }

   public ReferenceFrame getHeadCameraFrame()
   {
      return headCameraFrame;
   }

   public ReferenceFrame getSteppingCameraFrame()
   {
      return steppingCameraFrame;
   }

   public ReferenceFrame getObjectDetectionCameraFrame()
   {
      return objectDetectionCameraFrame;
   }

   public ReferenceFrame getHeadZED2CameraFrame()
   {
      return headZED2CameraFrame;
   }

   public ReferenceFrame getOusterLidarFrame()
   {
      return ousterLidarFrame;
   }
}
