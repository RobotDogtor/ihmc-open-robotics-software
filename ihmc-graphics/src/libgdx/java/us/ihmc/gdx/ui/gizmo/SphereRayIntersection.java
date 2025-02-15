package us.ihmc.gdx.ui.gizmo;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class SphereRayIntersection
{
   private final Sphere3D sphere = new Sphere3D();
   private final Point3D rayOriginInSphereFrame = new Point3D();
   private final Point3D firstIntersectionToPack = new Point3D();
   private final Point3D secondIntersectionToPack = new Point3D();

   public void setup(double radius, RigidBodyTransformReadOnly transform)
   {
      setup(radius, 0.0, transform);
   }

   public void setup(double radius, double zOffset, RigidBodyTransformReadOnly transform)
   {
      sphere.setToZero();
      sphere.setRadius(radius);
      sphere.getPosition().addZ(zOffset);
      sphere.applyTransform(transform);
   }

   public void setup(double radius, Point3DReadOnly offset, RigidBodyTransformReadOnly transform)
   {
      sphere.setToZero();
      sphere.setRadius(radius);
      sphere.getPosition().add(offset);
      sphere.applyTransform(transform);
   }

   public void setup(double radius, Point3DReadOnly positionInWorld)
   {
      sphere.setToZero();
      sphere.setRadius(radius);
      sphere.getPosition().set(positionInWorld);
   }

   public boolean intersect(Line3DReadOnly pickRay)
   {
      rayOriginInSphereFrame.setX(pickRay.getPoint().getX() - sphere.getPosition().getX());
      rayOriginInSphereFrame.setY(pickRay.getPoint().getY() - sphere.getPosition().getY());
      rayOriginInSphereFrame.setZ(pickRay.getPoint().getZ() - sphere.getPosition().getZ());
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(sphere.getRadius(),
                                                                                             sphere.getRadius(),
                                                                                             sphere.getRadius(),
                                                                                             rayOriginInSphereFrame,
                                                                                             pickRay.getDirection(),
                                                                                             firstIntersectionToPack,
                                                                                             secondIntersectionToPack);
      firstIntersectionToPack.add(sphere.getPosition());
      secondIntersectionToPack.add(sphere.getPosition());
      return numberOfIntersections == 2;
   }

   public Point3DReadOnly getFirstIntersectionToPack()
   {
      return firstIntersectionToPack;
   }

   public Point3DReadOnly getSecondIntersectionToPack()
   {
      return secondIntersectionToPack;
   }
}
