package us.ihmc.avatar.kinematicsSimulation;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WalkingStatusMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.logging.IntraprocessYoVariableLogger;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModule;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisHeightControlState;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameMessageCommandConverter;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfMassStateProvider;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class HumanoidKinematicsSimulation
{
   private static final double GRAVITY_Z = 9.81;
   private static final double LIDAR_SPINDLE_SPEED = 2.5;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final HumanoidKinematicsSimulationParameters kinematicsSimulationParameters;
   private final PausablePeriodicThread controlThread;
   private final ROS2Node ros2Node;
   private final RealtimeROS2Node realtimeROS2Node;
   private final IHMCROS2Publisher<RobotConfigurationData> robotConfigurationDataPublisher;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final SimulatedHandKinematicController simulatedHandKinematicController;
   private volatile RobotMotionStatus robotMotionStatus;
   private double yoVariableServerTime = 0.0;
   private final Stopwatch monotonicTimer = new Stopwatch();
   private final Stopwatch updateTimer = new Stopwatch();
   private final YoDouble yoTime;

   private final FullHumanoidRobotModel fullRobotModel;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final SideDependentList<SettableFootSwitch> footSwitches = new SideDependentList<>();

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WholeBodyControllerCore controllerCore;
   private final LinearMomentumRateControlModule linearMomentumRateControlModule;
   private final HighLevelControlManagerFactory managerFactory;
   private final WalkingHighLevelHumanoidController walkingController;

   private final CommandInputManager walkingInputManager = new CommandInputManager("walking_preview_internal",
                                                                                   ControllerAPIDefinition.getControllerSupportedCommands());
   private final StatusMessageOutputManager walkingOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());

   private final MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator();

   private AtomicReference<WalkingStatus> latestWalkingStatus = new AtomicReference<>();
   private SideDependentList<HumanoidKinematicsSimulationContactStateHolder> contactStateHolders = new SideDependentList<>();
   private InverseDynamicsCommandList inverseDynamicsContactHolderCommandList = new InverseDynamicsCommandList();
   private YoVariableServer yoVariableServer = null;
   private IntraprocessYoVariableLogger intraprocessYoVariableLogger;

   public static HumanoidKinematicsSimulation create(DRCRobotModel robotModel, HumanoidKinematicsSimulationParameters kinematicsSimulationParameters)
   {
      HumanoidKinematicsSimulation humanoidKinematicsSimulation = new HumanoidKinematicsSimulation(robotModel, kinematicsSimulationParameters);
      humanoidKinematicsSimulation.setRunning(true);
      return humanoidKinematicsSimulation;
   }

   public static HumanoidKinematicsSimulation createForPreviews(DRCRobotModel robotModel, HumanoidKinematicsSimulationParameters kinematicsSimulationParameters)
   {
      return new HumanoidKinematicsSimulation(robotModel, kinematicsSimulationParameters);
   }

   private HumanoidKinematicsSimulation(DRCRobotModel robotModel, HumanoidKinematicsSimulationParameters kinematicsSimulationParameters)
   {
      this.kinematicsSimulationParameters = kinematicsSimulationParameters;

      // instantiate some existing controller ROS2 API?
      ros2Node = ROS2Tools.createROS2Node(kinematicsSimulationParameters.getPubSubImplementation(), ROS2Tools.HUMANOID_KINEMATICS_CONTROLLER_NODE_NAME);

      robotConfigurationDataPublisher = new IHMCROS2Publisher<>(ros2Node,
                                                                RobotConfigurationData.class,
                                                                ROS2Tools.HUMANOID_CONTROLLER.withRobot(robotModel.getSimpleRobotName()).withOutput());

      String robotName = robotModel.getSimpleRobotName();
      fullRobotModel = robotModel.createFullRobotModel();
      CenterOfMassStateProvider centerOfMassStateProvider = CenterOfMassStateProvider.createJacobianBasedStateCalculator(fullRobotModel.getElevator(), worldFrame);
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel, centerOfMassStateProvider, null);

      // Create registries to match controller so the XML gets loaded properly.
      yoTime = new YoDouble("timeInPreview", registry);
      YoRegistry drcControllerThreadRegistry = new YoRegistry("DRCControllerThread");
      YoRegistry drcMomentumBasedControllerRegistry = new YoRegistry("DRCMomentumBasedController");
      YoRegistry humanoidHighLevelControllerManagerRegistry = new YoRegistry("HumanoidHighLevelControllerManager");
      YoRegistry managerParentRegistry = new YoRegistry("HighLevelHumanoidControllerFactory");
      YoRegistry walkingParentRegistry = new YoRegistry("WalkingControllerState");
      registry.addChild(drcControllerThreadRegistry);
      drcControllerThreadRegistry.addChild(drcMomentumBasedControllerRegistry);
      drcMomentumBasedControllerRegistry.addChild(humanoidHighLevelControllerManagerRegistry);
      humanoidHighLevelControllerManagerRegistry.addChild(walkingParentRegistry);
      humanoidHighLevelControllerManagerRegistry.addChild(managerParentRegistry);

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
      {
         contactableBodiesFactory.addAdditionalContactPoint(contactPointParameters.getAdditionalContactRigidBodyNames().get(i),
                                                            contactPointParameters.getAdditionalContactNames().get(i),
                                                            contactPointParameters.getAdditionalContactTransforms().get(i));
      }
      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);
      SideDependentList<ContactableFoot> feet = new SideDependentList<>(contactableBodiesFactory.createFootContactableFeet());
      List<ContactablePlaneBody> allContactableBodies = new ArrayList<>(contactableBodiesFactory.createAdditionalContactPoints());
      allContactableBodies.addAll(feet.values());
      contactableBodiesFactory.disposeFactory();

      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      for (RobotSide robotSide : RobotSide.values)
      {
         SettableFootSwitch footSwitch = new SettableFootSwitch(feet.get(robotSide), totalRobotWeight, 2, registry);
         footSwitch.setFootContactState(true);
         footSwitches.put(robotSide, footSwitch);
      }

      JointBasics[] jointsToIgnore = DRCControllerThread.createListOfJointsToIgnore(fullRobotModel, robotModel, robotModel.getSensorInformation());

      controllerToolbox = new HighLevelHumanoidControllerToolbox(fullRobotModel,
                                                                 centerOfMassStateProvider,
                                                                 referenceFrames,
                                                                 footSwitches,
                                                                 null,
                                                                 yoTime,
                                                                 GRAVITY_Z,
                                                                 robotModel.getWalkingControllerParameters().getOmega0(),
                                                                 feet,
                                                                 kinematicsSimulationParameters.getDt(),
                                                                 Collections.emptyList(),
                                                                 allContactableBodies,
                                                                 yoGraphicsListRegistry,
                                                                 jointsToIgnore);
      humanoidHighLevelControllerManagerRegistry.addChild(controllerToolbox.getYoVariableRegistry());
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      CoPTrajectoryParameters copTrajectoryParameters = robotModel.getCoPTrajectoryParameters();
      WalkingMessageHandler walkingMessageHandler = new WalkingMessageHandler(walkingControllerParameters.getDefaultTransferTime(),
                                                                              walkingControllerParameters.getDefaultSwingTime(),
                                                                              walkingControllerParameters.getDefaultInitialTransferTime(),
                                                                              walkingControllerParameters.getDefaultFinalTransferTime(),
                                                                              controllerToolbox.getContactableFeet(),
                                                                              walkingOutputManager,
                                                                              yoTime,
                                                                              yoGraphicsListRegistry,
                                                                              controllerToolbox.getYoVariableRegistry());
      controllerToolbox.setWalkingMessageHandler(walkingMessageHandler);
      controllerToolbox.attachRobotMotionStatusChangedListener((newStatus, time) -> robotMotionStatus = newStatus);

      // Initializes this desired robot to the most recent robot configuration data received from the walking controller.
      RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(kinematicsSimulationParameters.getInitialGroundHeight(),
                                                                                                                   kinematicsSimulationParameters.getInitialRobotYaw(),
                                                                                                                   kinematicsSimulationParameters.getInitialRobotX(),
                                                                                                                   kinematicsSimulationParameters.getInitialRobotY());
      robotInitialSetup.initializeFullRobotModel(fullRobotModel);

      managerFactory = new HighLevelControlManagerFactory(managerParentRegistry);
      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      managerFactory.setCopTrajectoryParameters(copTrajectoryParameters);
      managerFactory.setWalkingControllerParameters(walkingControllerParameters);

      walkingController = new WalkingHighLevelHumanoidController(walkingInputManager,
                                                                 walkingOutputManager,
                                                                 managerFactory,
                                                                 walkingControllerParameters,
                                                                 controllerToolbox);
      walkingParentRegistry.addChild(walkingController.getYoVariableRegistry());

      // create controller network subscriber here!!
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(kinematicsSimulationParameters.getPubSubImplementation(),
                                                          ROS2Tools.HUMANOID_KINEMATICS_CONTROLLER_NODE_NAME + "_rt");
      ROS2Topic inputTopic = ROS2Tools.getControllerInputTopic(robotName);
      ROS2Topic outputTopic = ROS2Tools.getControllerOutputTopic(robotName);
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(inputTopic,
                                                                                                walkingInputManager,
                                                                                                outputTopic,
                                                                                                walkingOutputManager,
                                                                                                realtimeROS2Node);
      controllerNetworkSubscriber.addMessageFilter(message ->
      {
         if (message instanceof FootstepDataListMessage)
         {
            FootstepDataListMessage footstepDataListMessage = (FootstepDataListMessage) message;
            footstepDataListMessage.setOffsetFootstepsHeightWithExecutionError(false); // fixes +Z drift for each step
         }
         return true;
      });
      walkingInputManager.registerConversionHelper(new FrameMessageCommandConverter(controllerToolbox.getReferenceFrameHashCodeResolver()));
      controllerNetworkSubscriber.registerSubcriberWithMessageUnpacker(WholeBodyTrajectoryMessage.class,
                                                                       9,
                                                                       MessageUnpackingTools.createWholeBodyTrajectoryMessageUnpacker());
      controllerNetworkSubscriber.addMessageCollectors(ControllerAPIDefinition.createDefaultMessageIDExtractor(), 3);
      controllerNetworkSubscriber.addMessageValidator(ControllerAPIDefinition.createDefaultMessageValidation());

      simulatedHandKinematicController = robotModel.createSimulatedHandKinematicController(fullRobotModel, realtimeROS2Node, yoTime);

      realtimeROS2Node.spin();

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(kinematicsSimulationParameters.getDt(),
                                                                                       GRAVITY_Z,
                                                                                       fullRobotModel.getRootJoint(),
                                                                                       controllerToolbox.getControlledJoints(),
                                                                                       controllerToolbox.getCenterOfMassFrame(),
                                                                                       walkingControllerParameters.getMomentumOptimizationSettings(),
                                                                                       yoGraphicsListRegistry,
                                                                                       registry);
      controlCoreToolbox.setJointPrivilegedConfigurationParameters(walkingControllerParameters.getJointPrivilegedConfigurationParameters());
      controlCoreToolbox.setFeedbackControllerSettings(walkingControllerParameters.getFeedbackControllerSettings());
      controlCoreToolbox.setupForInverseDynamicsSolver(controllerToolbox.getContactablePlaneBodies());

      controllerCore = new WholeBodyControllerCore(controlCoreToolbox,
                                                   managerFactory.createFeedbackControlTemplate(),
                                                   new JointDesiredOutputList(controllerToolbox.getControlledOneDoFJoints()),
                                                   walkingParentRegistry);
      walkingController.setControllerCoreOutput(controllerCore.getOutputForHighLevelController());

      linearMomentumRateControlModule = new LinearMomentumRateControlModule(centerOfMassStateProvider,
                                                                            referenceFrames,
                                                                            controllerToolbox.getContactableFeet(),
                                                                            fullRobotModel.getElevator(),
                                                                            walkingControllerParameters,
                                                                            GRAVITY_Z,
                                                                            controllerToolbox.getControlDT(),
                                                                            walkingParentRegistry,
                                                                            yoGraphicsListRegistry);

      ParameterLoaderHelper.loadParameters(this, robotModel, drcControllerThreadRegistry);

      YoVariable defaultHeight = registry.findVariable(PelvisHeightControlState.class.getSimpleName(),
                                                       PelvisHeightControlState.class.getSimpleName() + "DefaultHeight");
      if (Double.isNaN(defaultHeight.getValueAsDouble()))
      {
         throw new RuntimeException("Need to load a default height.");
      }

      if (kinematicsSimulationParameters.getLogToFile())
      {
         Path incomingLogsDirectory;
         intraprocessYoVariableLogger = new IntraprocessYoVariableLogger(getClass().getSimpleName(),
                                                                         robotModel.getLogModelProvider(),
                                                                         registry,
                                                                         fullRobotModel.getElevator(),
                                                                         yoGraphicsListRegistry,
                                                                         100000,
                                                                         0.01);
         intraprocessYoVariableLogger.start();
      }
      if (kinematicsSimulationParameters.getCreateYoVariableServer())
      {
         yoVariableServer = new YoVariableServer(getClass().getSimpleName(), robotModel.getLogModelProvider(), new DataServerSettings(false), 0.01);
         yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), yoGraphicsListRegistry);
         yoVariableServer.start();
      }

      walkingOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, this::processFootstepStatus);
      walkingOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, this::processWalkingStatus);

      controlThread = new PausablePeriodicThread(getClass().getSimpleName(), kinematicsSimulationParameters.getUpdatePeriod(), 5, this::controllerTick);
   }

   public void setState()
   {

   }

   public void setRunning(boolean running)
   {
      if (running && !controlThread.isRunning())
      {
         initialize();
         controlThread.start();
      }
      else if (!running && controlThread.isRunning())
      {
         controlThread.stop();
      }
   }

   public void initialize()
   {
      zeroMotion();

      referenceFrames.updateFrames();
      controllerCore.initialize();
      walkingController.initialize();

      walkingController.requestImmediateTransitionToStandingAndHoldCurrent();

      for (RobotSide robotSide : RobotSide.values)
      {
         contactStateHolders.put(robotSide,
                                 HumanoidKinematicsSimulationContactStateHolder.holdAtCurrent(controllerToolbox.getFootContactStates().get(robotSide)));
      }

      if (simulatedHandKinematicController != null)
         simulatedHandKinematicController.initialize();

      monotonicTimer.start();
   }

   public void controllerTick()
   {
      updateTimer.reset();

      doControl();

      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(fullRobotModel);
      robotConfigurationData.setMonotonicTime(Conversions.secondsToNanoseconds(yoTime.getValue()));
      robotConfigurationData.setRobotMotionStatus(robotMotionStatus.toByte());
      robotConfigurationDataPublisher.publish(robotConfigurationData);

      if (kinematicsSimulationParameters.runNoFasterThanMaxRealtimeRate())
      {
         while (updateTimer.totalElapsed() < kinematicsSimulationParameters.getDt() / kinematicsSimulationParameters.getMaxRealtimeRate())
            ThreadTools.sleep(1);
      }
   }

   private void doControl()
   {
      yoTime.add(kinematicsSimulationParameters.getDt());
      fullRobotModel.updateFrames();
      referenceFrames.updateFrames();
      controllerToolbox.update();

      inverseDynamicsContactHolderCommandList.clear();
      for (RobotSide side : contactStateHolders.sides())
      {
         contactStateHolders.get(side).doControl();
         inverseDynamicsContactHolderCommandList.addCommand(contactStateHolders.get(side).getOutput());
      }

      // Trigger footstep completion based on swing time alone
      if (contactStateHolders.sides().length == 1 && managerFactory.getOrCreateBalanceManager().isICPPlanDone())
      {
         footSwitches.get(contactStateHolders.sides()[0].getOppositeSide()).setFootContactState(true);
      }

      walkingController.doAction();

      linearMomentumRateControlModule.setInputFromWalkingStateMachine(walkingController.getLinearMomentumRateControlModuleInput());
      if (!linearMomentumRateControlModule.computeControllerCoreCommands())
      {
         controllerToolbox.reportControllerFailureToListeners(new FrameVector2D());
      }
      walkingController.setLinearMomentumRateControlModuleOutput(linearMomentumRateControlModule.getOutputForWalkingStateMachine());

      ControllerCoreCommand controllerCoreCommand = walkingController.getControllerCoreCommand();
      controllerCoreCommand.addInverseDynamicsCommand(linearMomentumRateControlModule.getMomentumRateCommand());
      controllerCoreCommand.addInverseDynamicsCommand(inverseDynamicsContactHolderCommandList);

      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      linearMomentumRateControlModule.setInputFromControllerCore(controllerCore.getControllerCoreOutput());
      linearMomentumRateControlModule.computeAchievedCMP();

      fullRobotModel.getRootJoint().setJointAcceleration(0, controllerCore.getOutputForRootJoint().getDesiredAcceleration());
      JointDesiredOutputListReadOnly jointDesiredOutputList = controllerCore.getOutputForLowLevelController();

      for (OneDoFJointBasics joint : controllerToolbox.getControlledOneDoFJoints())
      {
         JointDesiredOutputReadOnly jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         joint.setQdd(jointDesiredOutput.getDesiredAcceleration());
      }

      integrator.setIntegrationDT(kinematicsSimulationParameters.getDt());
      integrator.doubleIntegrateFromAcceleration(Arrays.asList(controllerToolbox.getControlledJoints()));

      // spin lidar
      JointBasics hokuyoJoint = fullRobotModel.getLidarJoint("head_hokuyo_sensor");
      if (hokuyoJoint instanceof RevoluteJoint)
      {
         RevoluteJoint revoluteHokuyoJoint = (RevoluteJoint) hokuyoJoint;
         revoluteHokuyoJoint.setQ(revoluteHokuyoJoint.getQ() + LIDAR_SPINDLE_SPEED * kinematicsSimulationParameters.getUpdatePeriod());
      }

      if (simulatedHandKinematicController != null)
         simulatedHandKinematicController.doControl();

      yoVariableServerTime += Conversions.millisecondsToSeconds(1);
      if (kinematicsSimulationParameters.getLogToFile())
      {
         intraprocessYoVariableLogger.update(Conversions.secondsToNanoseconds(yoVariableServerTime));
      }
      if (kinematicsSimulationParameters.getCreateYoVariableServer())
      {
         yoVariableServer.update(Conversions.secondsToNanoseconds(yoVariableServerTime));
      }
   }

   private void zeroMotion()
   {
      for (JointBasics joint : fullRobotModel.getElevator().childrenSubtreeIterable())
      {
         joint.setJointAccelerationToZero();
         joint.setJointTwistToZero();
      }
   }

   public static RobotConfigurationData extractRobotConfigurationData(FullHumanoidRobotModel fullRobotModel)
   {
      fullRobotModel.updateFrames();
      OneDoFJointBasics[] joints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      RobotConfigurationData robotConfigurationData = RobotConfigurationDataFactory.create(joints,
                                                                                           fullRobotModel.getForceSensorDefinitions(),
                                                                                           fullRobotModel.getIMUDefinitions());
      RobotConfigurationDataFactory.packJointState(robotConfigurationData, Arrays.stream(joints).collect(Collectors.toList()));
      robotConfigurationData.getRootTranslation().set(fullRobotModel.getRootJoint().getJointPose().getPosition());
      robotConfigurationData.getRootOrientation().set(fullRobotModel.getRootJoint().getJointPose().getOrientation());
      while (robotConfigurationData.getForceSensorData().size() < fullRobotModel.getForceSensorDefinitions().length) // pack empty force data
         robotConfigurationData.getForceSensorData().add();
      return robotConfigurationData;
   }

   private void processFootstepStatus(FootstepStatusMessage statusMessage)
   {
      RobotSide side = RobotSide.fromByte(statusMessage.getRobotSide());
      FootstepStatus status = FootstepStatus.fromByte(statusMessage.getFootstepStatus());
      FramePose3D desiredFootstep = new FramePose3D(worldFrame,
                                                    statusMessage.getDesiredFootPositionInWorld(),
                                                    statusMessage.getDesiredFootOrientationInWorld());

      switch (status)
      {
         case STARTED:
            contactStateHolders.remove(side);
            footSwitches.get(side).setFootContactState(false);
            break;
         case COMPLETED:
            contactStateHolders.put(side,
                                    new HumanoidKinematicsSimulationContactStateHolder(controllerToolbox.getFootContactStates().get(side), desiredFootstep));
            break;
         default:
            throw new RuntimeException("Unexpected status: " + status);
      }
   }

   private void processWalkingStatus(WalkingStatusMessage status)
   {
      latestWalkingStatus.set(WalkingStatus.fromByte(status.getWalkingStatus()));
   }

   public void destroy()
   {
      LogTools.info("Shutting down...");
      controlThread.destroy();
      ros2Node.destroy();
      realtimeROS2Node.destroy();
      if (yoVariableServer != null)
         yoVariableServer.close();
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }
}
