package us.ihmc.gdx.simulation.scs2;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.DynamicGDXModel;
import us.ihmc.log.LogTools;

import java.util.concurrent.atomic.AtomicInteger;

public class FrameGDXNodePart
{
   private static final AtomicInteger INDEX = new AtomicInteger();

   private final DynamicGDXModel model;
   private final ModelInstance modelInstance;
   private final String name;
   private final ReferenceFrame modelFrame;

   public FrameGDXNodePart(ReferenceFrame referenceFrame, DynamicGDXModel model, String name)
   {
      this.model = model;
      modelInstance = model.getOrCreateModelInstance();
      this.name = name;
      modelFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("modelFrame" + INDEX.getAndIncrement(),
                                                                                     referenceFrame,
                                                                                     model.getLocalTransform());
   }

   public void update()
   {
      try
      {
         RigidBodyTransform transformToRoot = modelFrame.getTransformToRoot();
         GDXTools.toGDX(transformToRoot, modelInstance.transform);
      }
      catch (NotARotationMatrixException e) // TODO: Why do we get this sometimes?
      {
         LogTools.warn(e.getMessage());
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }

   public void dispose()
   {
      model.dispose();
   }
}
