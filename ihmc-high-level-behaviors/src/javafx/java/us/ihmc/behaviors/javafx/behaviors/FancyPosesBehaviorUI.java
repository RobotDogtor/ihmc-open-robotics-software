package us.ihmc.behaviors.javafx.behaviors;

import javafx.fxml.FXML;
import javafx.scene.SubScene;
import javafx.scene.layout.Pane;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.javafx.JavaFXBehaviorUIDefinition;
import us.ihmc.behaviors.javafx.JavaFXBehaviorUIInterface;
import us.ihmc.behaviors.fancyPoses.FancyPosesBehavior;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2NodeInterface;

public class FancyPosesBehaviorUI extends JavaFXBehaviorUIInterface
{
   public static final JavaFXBehaviorUIDefinition DEFINITION = new JavaFXBehaviorUIDefinition(FancyPosesBehavior.DEFINITION, FancyPosesBehaviorUI::new);

   public FancyPosesBehaviorUI(SubScene sceneNode, Pane visualizationPane, ROS2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      super(sceneNode, visualizationPane, ros2Node, behaviorMessager, robotModel);
   }

   @Override
   public void setEnabled(boolean enabled)
   {

   }

   @FXML
   public void requestSingleSupport()
   {
      getBehaviorMessager().submitMessage(FancyPosesBehavior.API.GoToSingleSupport, true);
   }

   @FXML
   public void requestDoubleSupport()
   {
      getBehaviorMessager().submitMessage(FancyPosesBehavior.API.GoToDoubleSupport, true);
   }

   @FXML
   public void requestRunningMan()
   {
      getBehaviorMessager().submitMessage(FancyPosesBehavior.API.GoToRunningMan, false);
   }

   @FXML
   public void requestKarateKid1()
   {
      getBehaviorMessager().submitMessage(FancyPosesBehavior.API.GoToKarateKid1, false);
   }

   @FXML
   public void requestKarateKid2()
   {
      getBehaviorMessager().submitMessage(FancyPosesBehavior.API.GoToKarateKid2, false);
   }

   @FXML
   public void requestKarateKid3()
   {
      getBehaviorMessager().submitMessage(FancyPosesBehavior.API.GoToKarateKid3, false);
   }

   @FXML
   public void requestPresent()
   {
      getBehaviorMessager().submitMessage(FancyPosesBehavior.API.GoToPresent, false);
   }

   @FXML
   public void requestShutdownPose()
   {
      getBehaviorMessager().submitMessage(FancyPosesBehavior.API.GoToShutdownPose, false);
   }

   @Override
   public void destroy()
   {

   }
}
