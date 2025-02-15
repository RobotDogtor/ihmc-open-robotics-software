package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarPushRecoveryStandingTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasPushRecoveryStandingTest extends AvatarPushRecoveryStandingTest
{
   @Override
   public double getAngledPushMagnitude()
   {
      return 350.0;
   }

   @Override
   protected DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testControllerFailureKicksIn() throws SimulationExceededMaximumTimeException
   {
      super.testControllerFailureKicksIn();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testLongBackwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testLongBackwardPushWhileStanding();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testLongBackwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      setMagnitude(100.0);
      super.testLongBackwardPushWhileStandingAfterControllerFailureKickedIn();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testLongForwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testLongForwardPushWhileStanding();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testLongForwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      setMagnitude(100.0);
      super.testLongForwardPushWhileStandingAfterControllerFailureKickedIn();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileStanding();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushWhileStandingRecoveringAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      super.testPushWhileStandingRecoveringAfterControllerFailureKickedIn();
   }


   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testRecoveryWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      super.testRecoveryWhileInFlamingoStance();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testRecoveryForwardWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      super.testRecoveryForwardWhileInFlamingoStance();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testRecoverySidewaysWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      super.testRecoverySidewaysWhileInFlamingoStance();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testRecoveryAngledWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      super.testRecoveryAngledWhileInFlamingoStance();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testRecoveryPushForwardWhileInFlamingoStanceAndAfterTouchDown() throws SimulationExceededMaximumTimeException
   {
      super.testRecoveryPushForwardWhileInFlamingoStanceAndAfterTouchDown();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testFailureAfterRecoveryStep() throws SimulationExceededMaximumTimeException
   {
      super.testFailureAfterRecoveryStep();
   }
}
