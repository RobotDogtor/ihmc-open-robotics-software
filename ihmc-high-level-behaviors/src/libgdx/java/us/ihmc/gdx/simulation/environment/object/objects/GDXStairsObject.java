package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObjectFactory;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class GDXStairsObject extends GDXEnvironmentObject
{
   public static final String NAME = "Stairs";
   public static final GDXEnvironmentObjectFactory FACTORY = new GDXEnvironmentObjectFactory(NAME, GDXStairsObject.class);

   public GDXStairsObject()
   {
      super(NAME, FACTORY);
      Model realisticModel = GDXModelLoader.loadG3DModel("environmentObjects/stairs/Stairs.g3dj");
      setRealisticModel(realisticModel);

      double sizeX = 1.475;
      double sizeY = 2.355;
      double sizeZ = 1.072;
      setMass(500.0f);
      getCollisionShapeOffset().getTranslation().add(-sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0);
      getBoundingSphere().setRadius(5.0);
      getBoundingSphere().getPosition().add(sizeX / 2.0, sizeY / 2.0, sizeZ / 2.0);
      Box3D collisionBox = new Box3D(sizeX, sizeY, sizeZ);
      setCollisionModel(meshBuilder ->
      {
         Color color = GDXTools.toGDX(YoAppearance.LightSkyBlue());
         meshBuilder.addBox((float) sizeX, (float) sizeY, (float) sizeZ, color);
         meshBuilder.addMultiLineBox(collisionBox.getVertices(), 0.01, color); // some can see it better
      });
      setCollisionGeometryObject(collisionBox);

      // The stairs is multiple meshes I think, so more complicated; crashes currently; probably need to make a simpler mesh
//      btTriangleIndexVertexArray btTriangleIndexVertexArray = new btTriangleIndexVertexArray(realisticModel.meshParts);
//      btGImpactMeshShape btGImpactMeshShape = new btGImpactMeshShape(btTriangleIndexVertexArray);
//      btGImpactMeshShape.updateBound();
//      setBtCollisionShape(btGImpactMeshShape);
   }
}
