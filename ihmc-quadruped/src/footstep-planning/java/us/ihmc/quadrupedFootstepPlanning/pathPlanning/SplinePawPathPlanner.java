package us.ihmc.quadrupedFootstepPlanning.pathPlanning;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlanningResult;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SplinePawPathPlanner extends AbstractWaypointsForPawStepPlanner
{
   private static final int numberOfPoints = 5;

   private final YoPolynomial xPoly;
   private final YoPolynomial yPoly;
   private final YoPolynomial zPoly;
   private final YoPolynomial yawPoly;

   public SplinePawPathPlanner(YoRegistry registry)
   {
      this("", registry);
   }

   public SplinePawPathPlanner(String prefix, YoRegistry registry)
   {
      super(prefix, registry);

      xPoly = new YoPolynomial("xPoly", 4, registry);
      yPoly = new YoPolynomial("yPoly", 4, registry);
      zPoly = new YoPolynomial("zPoly", 4, registry);
      yawPoly = new YoPolynomial("yawPoly", 4, registry);
   }

   public PawStepPlanningResult planWaypoints()
   {
      waypoints.clear();
      double yaw = bodyStartPose.getYaw();
      xPoly.setQuadratic(0.0, 1.0, bodyStartPose.getX(), Math.cos(yaw) * 0.2, bodyGoalPose.getX());
      yPoly.setQuadratic(0.0, 1.0, bodyStartPose.getY(), Math.sin(yaw) * 0.2, bodyGoalPose.getY());
      zPoly.setQuadratic(0.0, 1.0, bodyStartPose.getZ(), 0.0, bodyGoalPose.getZ());
      yawPoly.setQuadratic(0.0, 1.0, bodyStartPose.getYaw(), 0.0, bodyGoalPose.getYaw());
      for (int i = 0; i < numberOfPoints; i++)
      {
         double percent = i / (double) (numberOfPoints - 1);
         xPoly.compute(percent);
         yPoly.compute(percent);
         zPoly.compute(percent);
         yawPoly.compute(percent);
         Pose3D pose = new Pose3D();
         pose.getPosition().set(xPoly.getValue(), yPoly.getValue(), zPoly.getValue());
         pose.getOrientation().setYawPitchRoll(yawPoly.getValue(), 0.0, 0.0);
         waypoints.add(pose);
      }

      yoResult.set(PawStepPlanningResult.OPTIMAL_SOLUTION);
      return yoResult.getEnumValue();
   }
}
