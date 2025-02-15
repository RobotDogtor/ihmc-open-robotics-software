package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.REASensorDataFilterParametersMessage;
import controller_msgs.msg.dds.REAStatusMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

public class REACurrentStateProvider
{
   private final IHMCROS2Publisher<REAStatusMessage> currentStatePublisher;
   private final AtomicReference<Boolean> isRunning, hasCleared, isUsingLidar, isUsingStereoVision, isUsingDepthCloud;
   private final AtomicReference<Double> minRange, maxRange;
   private final AtomicReference<BoundingBoxParametersMessage> boundingBoxParameters;
   private final REAStatusMessage currentState = new REAStatusMessage();

   public REACurrentStateProvider(ROS2NodeInterface ros2Node,
                                  ROS2Topic outputTopic,
                                  Messager messager)
   {
      currentStatePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, REAStatusMessage.class, outputTopic);
      isRunning = messager.createInput(REAModuleAPI.OcTreeEnable);
      // This should be the only input with a default value, the rest gets populated at the very start.
      hasCleared = messager.createInput(REAModuleAPI.OcTreeClear, false);
      isUsingLidar = messager.createInput(REAModuleAPI.LidarBufferEnable);
      isUsingStereoVision = messager.createInput(REAModuleAPI.StereoVisionBufferEnable);
      isUsingDepthCloud = messager.createInput(REAModuleAPI.DepthCloudBufferEnable);
      minRange = messager.createInput(REAModuleAPI.LidarMinRange);
      maxRange = messager.createInput(REAModuleAPI.LidarMaxRange);
      boundingBoxParameters = messager.createInput(REAModuleAPI.OcTreeBoundingBoxParameters);
   }

   public void publishCurrentState()
   {
      currentState.setIsRunning(isRunning.get());
      currentState.setIsUsingLidar(isUsingLidar.get());
      currentState.setIsUsingStereoVision(isUsingStereoVision.get());
      //currentState.setIsusingDepthCloud(isUsingDepthCloud.get()); // todo
      currentState.setHasCleared(hasCleared.getAndSet(false));
      REASensorDataFilterParametersMessage sensorFilterParameters = currentState.getCurrentSensorFilterParameters();
      sensorFilterParameters.setSensorMinRange(minRange.get());
      sensorFilterParameters.setSensorMaxRange(maxRange.get());
      sensorFilterParameters.getBoundingBoxMin().set(boundingBoxParameters.get().getMin());
      sensorFilterParameters.getBoundingBoxMax().set(boundingBoxParameters.get().getMax());
      currentStatePublisher.publish(currentState);
   }
}
