package us.ihmc.exampleSimulations.omniWrist;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.ExternalForcePointPDConstraintToIntegrate;

public class OmniWristSimulation
{

   public OmniWristSimulation()
   {
      OmniWristDescription description = new OmniWristDescription();
      RobotDescription robotDescription = description.createRobotDescription();

      RobotFromDescription robot = new RobotFromDescription(robotDescription);
//      Link.addEllipsoidFromMassPropertiesToAllLinks(robot, YoAppearance.Cornsilk());

      OmniWristController controller = new OmniWristController(robot);
      robot.setController(controller);

      YoDouble q_jointOneA = (YoDouble) robot.getRobotsYoRegistry().findVariable("q_jointOneA");
      YoDouble q_jointOneB = (YoDouble) robot.getRobotsYoRegistry().findVariable("q_jointOneB");
      YoDouble q_jointOneC = (YoDouble) robot.getRobotsYoRegistry().findVariable("q_jointOneC");
      YoDouble q_jointOneD = (YoDouble) robot.getRobotsYoRegistry().findVariable("q_jointOneD");
      YoDouble q_jointFourA = (YoDouble) robot.getRobotsYoRegistry().findVariable("q_jointFourA");

      double initialAngle = -0.38;
      q_jointOneA.set(initialAngle);
      q_jointOneB.set(initialAngle);
      q_jointOneC.set(initialAngle);
      q_jointOneD.set(initialAngle);
      q_jointFourA.set(-initialAngle);     

      YoRegistry registry = robot.getRobotsYoRegistry();

      double weldStiffness = 2000000.0;
      double weldDamping = 500.0;
      
      createWeldConstraint("weldB1", robot, weldStiffness, weldDamping, "ef_B1", "ef_matchB1", registry);
      createWeldConstraint("weldB2", robot, weldStiffness, weldDamping, "ef_B2", "ef_matchB2", registry);
      createWeldConstraint("weldC1", robot, weldStiffness, weldDamping, "ef_C1", "ef_matchC1", registry);
      createWeldConstraint("weldC2", robot, weldStiffness, weldDamping, "ef_C2", "ef_matchC2", registry);
      createWeldConstraint("weldD1", robot, weldStiffness, weldDamping, "ef_D1", "ef_matchD1", registry);
      createWeldConstraint("weldD2", robot, weldStiffness, weldDamping, "ef_D2", "ef_matchD2", registry);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(32000);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.00002, 100);
      scs.setSimulateNoFasterThanRealTime(true);
      scs.setGroundVisible(false);

      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.addModelFile(OmniWristDescription.omniWristBaseModelFile, YoAppearance.Black());
      scs.addStaticLinkGraphics(staticLinkGraphics);

      scs.setCameraFix(0.0, 0.15, 0.0);
      scs.setCameraPosition(0.0, -0.4, 0.13);

      scs.startOnAThread();
      scs.simulate();
   }

   private void createWeldConstraint(String weldName, RobotFromDescription robot, double weldStiffness, double weldDamping,
                                     String externalForcePointName, String externalForcePointMatchName, YoRegistry registry)
   {
      ExternalForcePoint ef_B1 = robot.getExternalForcePoint(externalForcePointName);
      ExternalForcePoint ef_matchB1 = robot.getExternalForcePoint(externalForcePointMatchName);
      ExternalForcePointPDConstraintToIntegrate weldJointB1 = new ExternalForcePointPDConstraintToIntegrate(weldName, ef_B1, ef_matchB1, registry);
      weldJointB1.setStiffness(weldStiffness);
      weldJointB1.setDamping(weldDamping);
      robot.addFunctionToIntegrate(weldJointB1);
   }

   public static void main(String[] args)
   {
      new OmniWristSimulation();
   }
}
