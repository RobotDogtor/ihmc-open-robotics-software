package us.ihmc.avatar;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.DO_NOTHING_BEHAVIOR;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.WALKING;

import java.util.ArrayList;

import javax.swing.JButton;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.factory.AvatarSimulationFactory;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.SplitFractionCalculatorParametersReadOnly;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.sensors.SixDOFForceTorqueSensorNameHolder;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class DRCFlatGroundWalkingTrack
{
   // looking for CREATE_YOVARIABLE_WALKING_PROVIDERS ?  use the second constructor and pass in WalkingProvider = YOVARIABLE_PROVIDER

   private final AvatarSimulation avatarSimulation;

   private final RealtimeROS2Node realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.FAST_RTPS, "flat_ground_walking_track_simulation");

   private static boolean createYoVariableServer = true;

   public DRCFlatGroundWalkingTrack(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup, DRCSCSInitialSetup scsInitialSetup,
                                    boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep, DRCRobotModel model)
   {
      this(robotInitialSetup, guiInitialSetup, scsInitialSetup, useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, model,
           WalkingProvider.VELOCITY_HEADING_COMPONENT, new HeadingAndVelocityEvaluationScriptParameters(), null, null, null); // should always be committed as VELOCITY_HEADING_COMPONENT
   }
   
   // SimpleAtlas Simulation calls this helper constructor
   public DRCFlatGroundWalkingTrack(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup, DRCSCSInitialSetup scsInitialSetup,
                                    boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep, DRCRobotModel model,
                                    HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters, HighLevelControllerStateFactory customControllerState,
                                    SixDOFForceTorqueSensorNameHolder sixDOFForceTorqueSensorNameHolder)
   {
      this(robotInitialSetup, guiInitialSetup, scsInitialSetup, useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, model,
           WalkingProvider.VELOCITY_HEADING_COMPONENT, walkingScriptParameters, null, customControllerState, sixDOFForceTorqueSensorNameHolder); // should always be committed as VELOCITY_HEADING_COMPONENT
   }
   
   public DRCFlatGroundWalkingTrack(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup, DRCSCSInitialSetup scsInitialSetup,
           boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep, DRCRobotModel model,
           HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
	{
		this(robotInitialSetup, guiInitialSetup, scsInitialSetup, useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, model,
				WalkingProvider.VELOCITY_HEADING_COMPONENT, walkingScriptParameters, null, null, null); // should always be committed as VELOCITY_HEADING_COMPONENT
	}

   public DRCFlatGroundWalkingTrack(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup,
                                    DRCSCSInitialSetup scsInitialSetup, boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep,
                                    DRCRobotModel model, PelvisPoseCorrectionCommunicatorInterface externalPelvisCorrectorSubscriber)
   {
      this(robotInitialSetup, guiInitialSetup, scsInitialSetup, useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, model,
           WalkingProvider.VELOCITY_HEADING_COMPONENT, new HeadingAndVelocityEvaluationScriptParameters(), externalPelvisCorrectorSubscriber, null, null);
   }

   public DRCFlatGroundWalkingTrack(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup,
                                    DRCSCSInitialSetup scsInitialSetup, boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep,
                                    DRCRobotModel model, WalkingProvider walkingProvider, HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
   {
      this(robotInitialSetup, guiInitialSetup, scsInitialSetup, useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, model, walkingProvider,
           walkingScriptParameters, null, null, null);
   }


   private DRCFlatGroundWalkingTrack(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup,
                                    DRCSCSInitialSetup scsInitialSetup, boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep,
                                    DRCRobotModel model, WalkingProvider walkingProvider, HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters,
                                    PelvisPoseCorrectionCommunicatorInterface externalPelvisCorrectorSubscriber, HighLevelControllerStateFactory customControllerStateFactory,
                                    SixDOFForceTorqueSensorNameHolder sixDOFForceTorqueSensorNameHolder)
   {
      //    scsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(model.getControllerDT() / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      HighLevelControllerParameters highLevelControllerParameters = model.getHighLevelControllerParameters();
      WalkingControllerParameters walkingControllerParameters = model.getWalkingControllerParameters();
      PushRecoveryControllerParameters pushRecoveryControllerParameters = model.getPushRecoveryControllerParameters();
      CoPTrajectoryParameters copTrajectoryParameters = model.getCoPTrajectoryParameters();
      SplitFractionCalculatorParametersReadOnly splitFractionParameters = model.getSplitFractionCalculatorParameters();
      HumanoidRobotSensorInformation sensorInformation = model.getSensorInformation();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      String[] forceSensorNames = sensorInformation.getForceSensorNames();
      ArrayList<String> sixDOFForceTorqueSensorNames = new ArrayList<>();
      for (String forceSensorName : forceSensorNames)
      {
    	  if (wristForceSensorNames!=null)
    		  if (!(feetForceSensorNames.containsValue(forceSensorName)) & !(wristForceSensorNames.containsValue(forceSensorName)))
    			  sixDOFForceTorqueSensorNames.add(forceSensorName);
      }
      
      // Added Network Processor
      HumanoidNetworkProcessorParameters networkProcessorParameters = new HumanoidNetworkProcessorParameters();
      networkProcessorParameters.setROSLocalhostURI();

      PacketCommunicator rosAPI_communicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_API_COMMUNICATOR, new IHMCCommunicationKryoNetClassList());

      networkProcessorParameters.setUseBehaviorModule(true, true);
      
      //

      RobotContactPointParameters<RobotSide> contactPointParameters = model.getContactPointParameters();
      ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
      ArrayList<String> additionalContactNames = contactPointParameters.getAdditionalContactNames();
      ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(), contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i), additionalContactNames.get(i), additionalContactTransforms.get(i));

      HighLevelHumanoidControllerFactory controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory, feetForceSensorNames, feetContactSensorNames,
                                                                                                    wristForceSensorNames, sixDOFForceTorqueSensorNames, highLevelControllerParameters,
                                                                                                    walkingControllerParameters,
                                                                                                    pushRecoveryControllerParameters,
                                                                                                    splitFractionParameters);
                                                                                                    copTrajectoryParameters);
      setupHighLevelStates(controllerFactory, feetForceSensorNames, highLevelControllerParameters.getFallbackControllerState());
      controllerFactory.setHeadingAndVelocityEvaluationScriptParameters(walkingScriptParameters);
      controllerFactory.createControllerNetworkSubscriber(model.getSimpleRobotName(), realtimeROS2Node);
      if (customControllerStateFactory != null)
         controllerFactory.addCustomControlState(customControllerStateFactory);
      controllerFactory.setSixDOFForceTorqueSensorNameHolder(sixDOFForceTorqueSensorNameHolder);;

      HeightMap heightMapForFootstepZ = null;
      if (cheatWithGroundHeightAtForFootstep)
      {
         heightMapForFootstepZ = scsInitialSetup.getHeightMap();
      }

      controllerFactory.createComponentBasedFootstepDataMessageGenerator(useVelocityAndHeadingScript, heightMapForFootstepZ);

      AvatarSimulationFactory avatarSimulationFactory = new AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(model);
      avatarSimulationFactory.setShapeCollisionSettings(model.getShapeCollisionSettings());
      avatarSimulationFactory.setHighLevelHumanoidControllerFactory(controllerFactory);
      avatarSimulationFactory.setCommonAvatarEnvironment(null);
      avatarSimulationFactory.setRobotInitialSetup(robotInitialSetup);
      avatarSimulationFactory.setSCSInitialSetup(scsInitialSetup);
      avatarSimulationFactory.setGuiInitialSetup(guiInitialSetup);
      avatarSimulationFactory.setRealtimeROS2Node(realtimeROS2Node);
      avatarSimulationFactory.setCreateYoVariableServer(createYoVariableServer);
      if (externalPelvisCorrectorSubscriber != null)
         avatarSimulationFactory.setExternalPelvisCorrectorSubscriber(externalPelvisCorrectorSubscriber);

      avatarSimulation = avatarSimulationFactory.createAvatarSimulation();
      initialize();

      if (robotInitialSetup.supportsReset())
      {
         JButton resetButton = new JButton("Reset Robot");
         resetButton.addActionListener(e -> avatarSimulation.resetRobot());
         avatarSimulation.getSimulationConstructionSet().addButton(resetButton);
      }

      avatarSimulation.getSimulationConstructionSet().setupVarGroup("kinematics", 
    		  new String[]{"t", "q_x", "q_y", "q_z", "q_qs", "q_qx", "q_qy", "q_qz",
    				  		"qd_x", "qd_y", "qd_z", "qd_wx", "qd_wy", "qd_wz"}, 
    		  new String[]{"q_to.*", "q_la.*", "q_ra.*", "q_ll.*", "q_rl.*", 
    				  		"qd_to.*", "qd_la.*", "qd_ra.*", "qd_ll.*", "qd_rl.*",
    				  		"tau_to.*", "tau_la.*", "tau_ra.*", "tau_ll.*", "tau_rl.*"});
      avatarSimulation.getSimulationConstructionSet().setupVarGroup("JointTorques", 
    		  new String[]{"t"}, 
    		  new String[]{"tau_to.*", "tau_la.*", "tau_ra.*", "tau_ll.*", "tau_rl.*"});
      avatarSimulation.getSimulationConstructionSet().setupVarGroup("JointAngles", 
    		  new String[]{"t", "q_x", "q_y", "q_z", "q_qs", "q_qx", "q_qy", "q_qz"}, 
    		  new String[]{"q_to.*", "q_la.*", "q_ra.*", "q_ll.*", "q_rl.*"});
      avatarSimulation.getSimulationConstructionSet().setupVarGroup("JointVelocities", 
    		  new String[]{"t", "qd_x", "qd_y", "qd_z", "qd_wx", "qd_wy", "qd_wz"}, 
    		  new String[]{"qd_to.*", "qd_la.*", "qd_ra.*", "qd_ll.*", "qd_rl.*"});
      avatarSimulation.start();
      realtimeROS2Node.spin();
   }

   /**
    * used to inject anything you need into scs before the sim starts
    */
   public void initialize()
   {

   }

   public void setupHighLevelStates(HighLevelHumanoidControllerFactory controllerFactory, SideDependentList<String> feetForceSensorNames,
                                    HighLevelControllerName fallbackControllerState)
   {
      controllerFactory.useDefaultDoNothingControlState();
      controllerFactory.useDefaultWalkingControlState();

      controllerFactory.addRequestableTransition(DO_NOTHING_BEHAVIOR, WALKING);
      controllerFactory.addRequestableTransition(WALKING, DO_NOTHING_BEHAVIOR);

      controllerFactory.addControllerFailureTransition(DO_NOTHING_BEHAVIOR, fallbackControllerState);
      controllerFactory.addControllerFailureTransition(WALKING, fallbackControllerState);

      controllerFactory.setInitialState(HighLevelControllerName.WALKING);
   }

   public void attachControllerFailureListener(ControllerFailureListener listener)
   {
      avatarSimulation.getHighLevelHumanoidControllerFactory().attachControllerFailureListener(listener);
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return avatarSimulation.getSimulationConstructionSet();
   }

   public AvatarSimulation getAvatarSimulation()
   {
      return avatarSimulation;
   }

   public void destroySimulation()
   {
      if (avatarSimulation != null)
      {
         avatarSimulation.dispose();
      }
   }
}
