package us.ihmc.behaviors.tools.behaviorTree;

/**
 * Core building block of behavior trees. The status that gets passed up the tree.
 */
public enum BehaviorTreeNodeStatus
{
   RUNNING,
   FAILURE,
   SUCCESS
}
