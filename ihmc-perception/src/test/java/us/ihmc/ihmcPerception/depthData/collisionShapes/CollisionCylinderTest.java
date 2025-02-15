package us.ihmc.ihmcPerception.depthData.collisionShapes;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;

public class CollisionCylinderTest
{
   @Test
   public void testInsideCylinderPoints()
   {
      Random random = new Random(98124L);

      CollisionCylinder cylinder = new CollisionCylinder(new RigidBodyTransform(), 5.0, 10.0);
      for (int i = 0; i < 1000000; i++)
      {
         double radius = -5.0 + 10.0 * random.nextDouble();
         Vector2D vector = new Vector2D(random.nextDouble(), random.nextDouble());
         vector.normalize();
         vector.scale(radius);
         
         double z = -5.0 +  10.0 * random.nextDouble();  
         
         
         Point3D point = new Point3D(vector.getX(), vector.getY(), z);
         assertTrue(cylinder.contains(point));
      }
   }
   @Test
   public void testOutideCylinderPoints()
   {
      Random random = new Random(98716L);
      
      CollisionCylinder cylinder = new CollisionCylinder(new RigidBodyTransform(), 5.0, 10.0);
      for (int i = 0; i < 1000000; i++)
      {
         double radius = (random.nextBoolean()?1.0:-1.0) * (5.0 + 10.0 * random.nextDouble());
         Vector2D vector = new Vector2D(random.nextDouble(), random.nextDouble());
         vector.normalize();
         vector.scale(radius);
         
         double z = (random.nextBoolean()?1.0:-1.0) * (5.0 + 10.0 * random.nextDouble());  
         
         
         Point3D point = new Point3D(vector.getX(), vector.getY(), z);
         assertFalse(cylinder.contains(point));
      }
   }
   
   
}
