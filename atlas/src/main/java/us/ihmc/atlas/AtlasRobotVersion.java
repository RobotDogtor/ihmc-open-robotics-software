package us.ihmc.atlas;

import java.io.InputStream;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.DRCHandType;

public enum AtlasRobotVersion
{
   ATLAS_UNPLUGGED_V5_NO_FOREARMS,
   ATLAS_UNPLUGGED_V5_NO_HANDS,
   ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ,
   ATLAS_UNPLUGGED_V5_ROBOTIQ_AND_SRI,
   ATLAS_UNPLUGGED_V5_TROOPER;

   private static String[] resourceDirectories;
   private final SideDependentList<RigidBodyTransform> offsetHandFromAttachmentPlate = new SideDependentList<RigidBodyTransform>();

   public DRCHandType getHandModel()
   {
      switch (this)
      {
      case ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ:
      case ATLAS_UNPLUGGED_V5_TROOPER:
         return DRCHandType.ROBOTIQ;
      case ATLAS_UNPLUGGED_V5_ROBOTIQ_AND_SRI:
         return DRCHandType.ROBOTIQ_AND_SRI;
      case ATLAS_UNPLUGGED_V5_NO_HANDS:
      case ATLAS_UNPLUGGED_V5_NO_FOREARMS:
      default:
         return DRCHandType.NONE;
      }
   }

   public boolean hasRobotiqHands()
   {
      return getHandModel() == DRCHandType.ROBOTIQ || getHandModel() == DRCHandType.ROBOTIQ_AND_SRI;
   }

   public String getSdfFile()
   {
      switch (this)
      {
      case ATLAS_UNPLUGGED_V5_NO_HANDS:
         return "models/GFE/atlas_unplugged_v5.sdf";
      case ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ:
      case ATLAS_UNPLUGGED_V5_ROBOTIQ_AND_SRI:
         return "models/GFE/atlas_unplugged_v5_dual_robotiq.sdf";
      case ATLAS_UNPLUGGED_V5_NO_FOREARMS:
         return "models/GFE/atlas_unplugged_v5_no_forearms.sdf";
      case ATLAS_UNPLUGGED_V5_TROOPER:
         return "models/GFE/atlas_unplugged_v5_trooper.sdf";
      default:
         throw new RuntimeException("AtlasRobotVersion: Unimplemented enumeration case : " + this);
      }
   }

   public String[] getResourceDirectories()
   {
      if (resourceDirectories == null)
      {
         resourceDirectories = new String[] {"models/GFE/"};
      }
      return resourceDirectories;
   }

   public InputStream getSdfFileAsStream()
   {
      return getClass().getClassLoader().getResourceAsStream(getSdfFile());
   }

   public RigidBodyTransform getOffsetFromAttachmentPlate(RobotSide side)
   {
      if (offsetHandFromAttachmentPlate.get(side) == null)
      {
         createTransforms();
      }
      return offsetHandFromAttachmentPlate.get(side);
   }

   private void createTransforms()
   {
      double distanceAttachmentPlateHand;
      
      switch (this)
      {
      case ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ:
      case ATLAS_UNPLUGGED_V5_ROBOTIQ_AND_SRI:
         distanceAttachmentPlateHand = 0.12; // On the palm.
         break;
      default:
         distanceAttachmentPlateHand = 0.0;
         break;
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         Point3D centerOfHandToWristTranslation = new Point3D();
         Quaternion centerOfHandToWristOrientation = new Quaternion();

         if (hasRobotiqHands())
         {
            centerOfHandToWristTranslation = new Point3D(distanceAttachmentPlateHand, 0.0, 0.0);
            centerOfHandToWristOrientation.appendRollRotation(robotSide.negateIfLeftSide(Math.PI * 0.5));
         }
         offsetHandFromAttachmentPlate.set(robotSide, new RigidBodyTransform(centerOfHandToWristOrientation, centerOfHandToWristTranslation));
      }
   }

   public String getModelName()
   {
      return "atlas";
   }
}
