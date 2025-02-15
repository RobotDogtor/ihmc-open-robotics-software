package us.ihmc.avatar.scs2;

import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.AvatarEstimatorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.avatar.factory.DisposableRobotController;
import us.ihmc.avatar.logging.IntraprocessYoVariableLogger;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.SessionState;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;

public class SCS2AvatarSimulation
{
   private Robot robot;
   private SimulationSession simulationSession;
   private HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory;
   private YoVariableServer yoVariableServer;
   private IntraprocessYoVariableLogger intraprocessYoVariableLogger;
   private DisposableRobotController robotController;
   private AvatarEstimatorThread estimatorThread;
   private AvatarControllerThread controllerThread;
   private SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider;
   private FullHumanoidRobotModel controllerFullRobotModel;
   private DRCRobotModel robotModel;
   private boolean showGUI;
   private boolean automaticallyStartSimulation;
   private RealtimeROS2Node realtimeROS2Node;

   private boolean systemExitOnDestroy = true;
   private boolean javaFXThreadImplicitExit = true;
   private SessionVisualizerControls sessionVisualizerControls;

   private boolean hasBeenDestroyed = false;

   public void start()
   {
      beforeSessionThreadStart();

      simulationSession.setSessionState(SessionState.ACTIVE);

      if (showGUI)
      {
         sessionVisualizerControls = SessionVisualizer.startSessionVisualizer(simulationSession, javaFXThreadImplicitExit);
         sessionVisualizerControls.addVisualizerShutdownListener(this::destroy);
      }
      if (automaticallyStartSimulation)
         simulationSession.setSessionMode(SessionMode.RUNNING);

      afterSessionThreadStart();
   }

   public void beforeSessionThreadStart()
   {
      if (intraprocessYoVariableLogger != null)
      {
         intraprocessYoVariableLogger.start();
      }
      if (yoVariableServer != null)
      {
         yoVariableServer.start();
      }
   }

   public void afterSessionThreadStart()
   {
      if (realtimeROS2Node != null)
         realtimeROS2Node.spin();
   }

   public void destroy()
   {
      if (hasBeenDestroyed)
         return;

      LogTools.info("Destroying simulation");
      hasBeenDestroyed = true;

      if (robotController != null)
      {
         robotController.dispose();
         robotController = null;
      }

      if (yoVariableServer != null)
      {
         yoVariableServer.close();
         yoVariableServer = null;
      }

      if (realtimeROS2Node != null)
      {
         realtimeROS2Node.destroy();
         realtimeROS2Node = null;
      }

      if (simulationSession != null)
      {
         if (showGUI)
            sessionVisualizerControls.shutdownNow();
         simulationSession.shutdownSession();
         simulationSession = null;
      }

      if (systemExitOnDestroy)
      {
         // TODO Remove this when pub-sub is released with the IntraProcessDomainImpl threads setup as daemon.
         System.exit(0);
      }
   }

   public boolean hasBeenDestroyed()
   {
      return hasBeenDestroyed;
   }

   public SimulationSession getSimulationSession()
   {
      return simulationSession;
   }

   public SessionVisualizerControls getSessionVisualizerControls()
   {
      return sessionVisualizerControls;
   }

   public FullHumanoidRobotModel getControllerFullRobotModel()
   {
      return controllerFullRobotModel;
   }

   public void setSystemExitOnDestroy(boolean systemExitOnDestroy)
   {
      this.systemExitOnDestroy = systemExitOnDestroy;
   }

   public void setJavaFXThreadImplicitExit(boolean javaFXThreadImplicitExit)
   {
      this.javaFXThreadImplicitExit = javaFXThreadImplicitExit;
   }

   /**
    * For unit testing only
    */
   public void addRobotControllerOnControllerThread(RobotController controller)
   {
      controllerThread.addRobotController(controller);
   }

   public FullRobotModelCorruptor getFullRobotModelCorruptor()
   {
      return controllerThread.getFullRobotModelCorruptor();
   }

   public HighLevelHumanoidControllerFactory getHighLevelHumanoidControllerFactory()
   {
      return highLevelHumanoidControllerFactory;
   }

   public SimulatedDRCRobotTimeProvider getSimulatedRobotTimeProvider()
   {
      return simulatedRobotTimeProvider;
   }

   public void setSimulationSession(SimulationSession simulationSession)
   {
      this.simulationSession = simulationSession;
   }

   public void setHighLevelHumanoidControllerFactory(HighLevelHumanoidControllerFactory momentumBasedControllerFactory)
   {
      this.highLevelHumanoidControllerFactory = momentumBasedControllerFactory;
   }

   public void setYoVariableServer(YoVariableServer yoVariableServer)
   {
      this.yoVariableServer = yoVariableServer;
   }

   public void setIntraprocessYoVariableLogger(IntraprocessYoVariableLogger intraprocessYoVariableLogger)
   {
      this.intraprocessYoVariableLogger = intraprocessYoVariableLogger;
   }

   public void setRobotController(DisposableRobotController robotController)
   {
      this.robotController = robotController;
   }

   public void setEstimatorThread(AvatarEstimatorThread estimatorThread)
   {
      this.estimatorThread = estimatorThread;
   }

   public AvatarEstimatorThread getEstimatorThread()
   {
      return estimatorThread;
   }

   public void setControllerThread(AvatarControllerThread controllerThread)
   {
      this.controllerThread = controllerThread;
   }

   public AvatarControllerThread getControllerThread()
   {
      return controllerThread;
   }

   public void setSimulatedRobotTimeProvider(SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider)
   {
      this.simulatedRobotTimeProvider = simulatedRobotTimeProvider;
   }

   public void setFullHumanoidRobotModel(FullHumanoidRobotModel controllerFullRobotModel)
   {
      this.controllerFullRobotModel = controllerFullRobotModel;
   }

   public void addRobotControllerOnEstimatorThread(RobotController controller)
   {
      estimatorThread.addRobotController(controller);
   }

   public void setRobot(Robot robot)
   {
      this.robot = robot;
   }

   public Robot getRobot()
   {
      return robot;
   }

   public RobotDefinition getRobotDefinition()
   {
      return robot.getRobotDefinition();
   }

   public void setRobotModel(DRCRobotModel robotModel)
   {
      this.robotModel = robotModel;
   }

   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   public void setShowGUI(boolean showGUI)
   {
      this.showGUI = showGUI;
   }

   public boolean getShowGUI()
   {
      return showGUI;
   }

   public void setAutomaticallyStartSimulation(boolean automaticallyStartSimulation)
   {
      this.automaticallyStartSimulation = automaticallyStartSimulation;
   }

   public void setRealTimeROS2Node(RealtimeROS2Node realtimeROS2Node)
   {
      this.realtimeROS2Node = realtimeROS2Node;
   }
}
