package us.ihmc.valkyrie.obstacleCourse;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseDoNothingTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@Tag("fast")
public class ValkyrieObstacleCourseDoNothingTest extends DRCObstacleCourseDoNothingTest
{
   private ValkyrieRobotModel robotModel;

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Tag("humanoid-obstacle")
   @Test
   public void testDoNothingGroundContactPoints() throws SimulationExceededMaximumTimeException
   {
      robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      super.testDoNothing1();
   }

   @Disabled // FIXME: Shape collision is not working
   @Test
   public void testDoNothingShapeCollision() throws SimulationExceededMaximumTimeException
   {
      robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      super.testDoNothing1();
   }
}
