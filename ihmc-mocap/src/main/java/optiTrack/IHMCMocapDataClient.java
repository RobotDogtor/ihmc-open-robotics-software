package optiTrack;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class IHMCMocapDataClient extends MocapDataClient
{
   private ReferenceFrame mocapOriginFrame;
   private ReferenceFrame mocapRbFrame;
   private ReferenceFrame mocapRbZUpFrame;

   private RigidBodyTransform mocapRbToMocapOrigin;

   public IHMCMocapDataClient()
   {
      super();

      setupReferenceFrames();
   }

   public void setupReferenceFrames()
   {
      mocapOriginFrame = new ReferenceFrame("mocapOrigin", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setRotationEulerAndZeroTranslation(Math.toRadians(-90), Math.toRadians(0), Math.toRadians(0));
         }
      };
      mocapOriginFrame.update();

      mocapRbFrame = new ReferenceFrame("mocapRB", mocapOriginFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(mocapRbToMocapOrigin);
         }
      };

      mocapRbZUpFrame = new ReferenceFrame("mocapRbZUpFrame", mocapRbFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setRotationEulerAndZeroTranslation(Math.toRadians(90), 0, 0);
         }
      };
      mocapRbZUpFrame.update();
   }

   @Override
   protected void updateListeners(ArrayList<MocapRigidBody> lisftOfRigidbodies)
   {
      ArrayList<MocapRigidBody> convertedListOfMocapRigidBodies = new ArrayList<MocapRigidBody>();
      for (MocapRigidBody mocapRigidBody : lisftOfRigidbodies)
      {
         mocapRbToMocapOrigin = new RigidBodyTransform(new Quaternion(mocapRigidBody.qx, mocapRigidBody.qy, mocapRigidBody.qz, mocapRigidBody.qw),
                 new Vector3D(mocapRigidBody.xPosition, mocapRigidBody.yPosition, mocapRigidBody.zPosition));
         mocapRbFrame.update();
         mocapOriginFrame.update();
         mocapRbZUpFrame.update();
         
         FramePose3D pose = new FramePose3D(mocapRbZUpFrame, new Point3D(),
                                        new Quaternion());
         pose.changeFrame(ReferenceFrame.getWorldFrame());

         RigidBodyTransform r = new RigidBodyTransform();
         pose.get(r);

         Vector3D position = new Vector3D();
         position.set(r.getTranslation());

         Quaternion rotation = new Quaternion();
         rotation.set(r.getRotation());

         convertedListOfMocapRigidBodies.add(new MocapRigidBody(mocapRigidBody.getId(), position, rotation, mocapRigidBody.getListOfAssociatedMarkers(),
                 mocapRigidBody.dataValid));
      }

      updateRigidBodiesListeners(convertedListOfMocapRigidBodies);
   }

   public static void main(String args[])
   {
      IHMCMocapDataClient udpMulticastClient = new IHMCMocapDataClient();
   }
}


//~ Formatted by Jindent --- http://www.jindent.com
