package us.ihmc.behaviors.javafx.tools;

import java.util.concurrent.ExecutorService;

import javafx.scene.Group;
import javafx.scene.transform.Translate;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.factory.RobotDefinitionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.javafx.PrivateAnimationTimer;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.simulationConstructionSetTools.grahics.GraphicsIDRobot;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.tools.thread.Activator;

public class JavaFXRemoteRobotVisualizer extends Group
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ExecutorService executor;

   private GraphicsRobot graphicsRobot;
   private JavaFXGraphics3DNode robotRootNode;
   private Activator robotLoadedActivator = new Activator();
   private boolean trackRobot = false;
   private FocusBasedCameraMouseEventHandler cameraForOptionalTracking;
   private Translate robotTranslate;
   private Translate savedCameraTranslate;
   private boolean hasPrepended = false;

   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);

   public JavaFXRemoteRobotVisualizer(DRCRobotModel robotModel, ROS2NodeInterface ros2Node)
   {
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);

      executor = ThreadTools.newSingleDaemonThreadExecutor("RobotVisualizerLoading");
      executor.submit(() -> loadRobotModelAndGraphics(robotModel.getRobotDefinition()));

      animationTimer.start();
   }

   public void setTrackRobot(FocusBasedCameraMouseEventHandler camera, boolean trackRobot)
   {
      this.trackRobot = trackRobot;
      cameraForOptionalTracking = camera;
      if (!hasPrepended)
      {
         hasPrepended = true;
         robotTranslate = new Translate();
         cameraForOptionalTracking.prependTransform(robotTranslate);
      }
      else if (trackRobot)
      {
         cameraForOptionalTracking.getTranslate().setX(savedCameraTranslate.getX());
         cameraForOptionalTracking.getTranslate().setY(savedCameraTranslate.getY());
         cameraForOptionalTracking.getTranslate().setZ(savedCameraTranslate.getZ());
      }
      else // !trackRobot
      {
         savedCameraTranslate = cameraForOptionalTracking.getTranslate().clone();
      }
   }

   private void handle(long now)
   {
      if (robotLoadedActivator.poll())
      {
         if (robotLoadedActivator.hasChanged())
         {
            getChildren().add(robotRootNode);
         }

         syncedRobot.update();

         graphicsRobot.update();
         robotRootNode.update();

         if (trackRobot)
         {
            FramePose3DReadOnly walkingFrame = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame);

            robotTranslate.setX(walkingFrame.getPosition().getX());
            robotTranslate.setY(walkingFrame.getPosition().getY());
            robotTranslate.setZ(walkingFrame.getPosition().getZ());
         }
      }
   }

   private void loadRobotModelAndGraphics(RobotDefinition robotDefinition)
   {
      graphicsRobot = new GraphicsIDRobot(robotDefinition.getName(), syncedRobot.getFullRobotModel().getElevator(), RobotDefinitionTools.toGraphicsObjectsHolder(robotDefinition));
      robotRootNode = new JavaFXGraphics3DNode(graphicsRobot.getRootNode());
      robotRootNode.setMouseTransparent(true);
      addNodesRecursively(graphicsRobot.getRootNode(), robotRootNode);
      robotRootNode.update();

      robotLoadedActivator.activate();
   }

   private void addNodesRecursively(Graphics3DNode graphics3dNode, JavaFXGraphics3DNode parentNode)
   {
      JavaFXGraphics3DNode node = new JavaFXGraphics3DNode(graphics3dNode);
      parentNode.addChild(node);
      graphics3dNode.getChildrenNodes().forEach(child -> addNodesRecursively(child, node));
   }

   public void destroy()
   {
      executor.shutdownNow();
   }
}
