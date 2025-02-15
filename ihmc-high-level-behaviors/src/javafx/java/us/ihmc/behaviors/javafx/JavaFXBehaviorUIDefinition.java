package us.ihmc.behaviors.javafx;

import us.ihmc.behaviors.BehaviorDefinition;

public class JavaFXBehaviorUIDefinition extends BehaviorDefinition
{
   private final JavaFXBehaviorUIInterfaceConstructor behaviorUISupplier;

   public JavaFXBehaviorUIDefinition(BehaviorDefinition definition, JavaFXBehaviorUIInterfaceConstructor behaviorUISupplier)
   {
      super(definition.getName(),
            definition.getBehaviorSupplier(),
            definition.getBehaviorAPI(),
            definition.getSubBehaviors().toArray(new BehaviorDefinition[0]));
      this.behaviorUISupplier = behaviorUISupplier;
   }

   public JavaFXBehaviorUIInterfaceConstructor getBehaviorUISupplier()
   {
      return behaviorUISupplier;
   }
}
