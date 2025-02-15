package us.ihmc.gdx.ui.missionControl.processes;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.gdx.ui.missionControl.RestartableMissionControlProcess;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

import java.util.Map;

public class KinematicsStreamingToolboxProcess extends RestartableMissionControlProcess
{
   private final DRCRobotModel robotModel;
   private Map<String, Double> initialConfiguration;
   private KinematicsStreamingToolboxModule kinematicsStreamingToolboxModule;

   public KinematicsStreamingToolboxProcess(DRCRobotModel robotModel, Map<String, Double> initialConfiguration)
   {
      this.robotModel = robotModel;
      this.initialConfiguration = initialConfiguration;
   }

   @Override
   protected void startInternal()
   {
      boolean startYoVariableServer = true;
      kinematicsStreamingToolboxModule = new KinematicsStreamingToolboxModule(robotModel, startYoVariableServer, PubSubImplementation.FAST_RTPS);
      KinematicsStreamingToolboxController controller = (KinematicsStreamingToolboxController) kinematicsStreamingToolboxModule.getToolboxController();
      controller.setInitialRobotConfigurationNamedMap(initialConfiguration);
      controller.getTools().getIKController().getCenterOfMassSafeMargin().set(0.10);
      kinematicsStreamingToolboxModule.wakeUp();
   }

   @Override
   protected void stopInternal()
   {
      kinematicsStreamingToolboxModule.destroy();
   }

   @Override
   public String getName()
   {
      return "Kinematics Streaming Toolbox";
   }

   public KinematicsStreamingToolboxModule getKinematicsStreamingToolboxModule()
   {
      return kinematicsStreamingToolboxModule;
   }
}
