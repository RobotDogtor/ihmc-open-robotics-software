package us.ihmc.robotics.screwTheory;

import java.util.stream.Stream;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;

public class MomentumCalculator
{
   private final Momentum tempMomentum = new Momentum();
   private final RigidBodyBasics[] rigidBodiesInOrders;

   public MomentumCalculator(RigidBodyBasics... rigidBodies)
   {
      rigidBodiesInOrders = Stream.of(rigidBodies).filter(body -> body.getInertia() != null).toArray(RigidBodyBasics[]::new);
   }

   public MomentumCalculator(RigidBodyBasics rootBody)
   {
      this(rootBody.subtreeArray());
   }

   public void computeAndPack(Momentum momentum)
   {
      momentum.setToZero();

      for (RigidBodyBasics rigidBody : rigidBodiesInOrders)
      {
         SpatialInertiaBasics inertia = rigidBody.getInertia();
         tempMomentum.setReferenceFrame(inertia.getReferenceFrame());
         tempMomentum.compute(inertia, rigidBody.getBodyFixedFrame().getTwistOfFrame());
         tempMomentum.changeFrame(momentum.getReferenceFrame());
         momentum.add(tempMomentum);
      }
   }
}
