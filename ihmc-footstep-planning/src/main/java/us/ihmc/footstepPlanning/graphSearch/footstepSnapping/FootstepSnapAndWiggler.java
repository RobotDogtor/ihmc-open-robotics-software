package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.commonWalkingControlModules.polygonWiggling.*;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.heightMap.HeightMapData;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashMap;
import java.util.function.ToDoubleFunction;

public class FootstepSnapAndWiggler implements FootstepSnapperReadOnly
{
   private final SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;
   private final FootstepPlannerParametersReadOnly parameters;

   private final GradientDescentStepConstraintSolver gradientDescentStepConstraintSolver;
   private final WiggleParameters wiggleParameters = new WiggleParameters();
   private final PlanarRegion planarRegionToPack = new PlanarRegion();
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private final Cylinder3D legCollisionShape = new Cylinder3D();
   private final RigidBodyTransform legCollisionShapeToSoleTransform = new RigidBodyTransform();
   private final RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();
   private final GradientDescentStepConstraintInput gradientDescentStepConstraintInput = new GradientDescentStepConstraintInput();
   private double flatGroundHeight = 0.0;

   private final HashMap<DiscreteFootstep, FootstepSnapData> snapDataHolder = new HashMap<>();
   protected PlanarRegionsList planarRegionsList;

   private HeightMapData heightMapData;
   private final HeightMapPolygonSnapper heightMapSnapper = new HeightMapPolygonSnapper();

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   // Use this by default
   public FootstepSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, FootstepPlannerParametersReadOnly parameters)
   {
      this(footPolygonsInSoleFrame, parameters, null, null, null);
   }

   // Call this constructor only for testing
   public FootstepSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame,
                                 FootstepPlannerParametersReadOnly parameters,
                                 TickAndUpdatable tickAndUpdatable,
                                 YoGraphicsListRegistry graphicsListRegistry,
                                 YoRegistry parentRegistry)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      this.parameters = parameters;

      if (tickAndUpdatable == null)
      {
         gradientDescentStepConstraintSolver = new GradientDescentStepConstraintSolver();
      }
      else
      {
         gradientDescentStepConstraintSolver = new GradientDescentStepConstraintSolver(tickAndUpdatable, graphicsListRegistry, parentRegistry);
      }
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
      snapDataHolder.clear();
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
   }

   public void setFlatGroundHeight(double flatGroundHeight)
   {
      this.flatGroundHeight = flatGroundHeight;
   }

   public void initialize()
   {
      updateWiggleParameters(wiggleParameters, parameters);
   }

   private boolean flatGroundMode()
   {
      return (planarRegionsList == null || planarRegionsList.isEmpty()) && heightMapData == null;
   }

   public FootstepSnapData snapFootstep(DiscreteFootstep footstep)
   {
      return snapFootstep(footstep, null, false);
   }

   @Override
   public FootstepSnapData snapFootstep(DiscreteFootstep footstep, DiscreteFootstep stanceStep, boolean computeWiggleTransform)
   {
      if (snapDataHolder.containsKey(footstep))
      {
         FootstepSnapData snapData = snapDataHolder.get(footstep);
         if (snapData.getSnapTransform().containsNaN())
         {
            return snapData;
         }
         else if (snapData.getWiggleTransformInWorld().containsNaN() && computeWiggleTransform)
         {
            computeWiggleTransform(footstep, stanceStep, snapData);
         }

         return snapData;
      }
      else if (flatGroundMode())
      {
         return FootstepSnapData.identityData(flatGroundHeight);
      }
      else
      {
         FootstepSnapData snapData = computeSnapTransform(footstep, stanceStep);
         snapDataHolder.put(footstep, snapData);

         if (snapData.getSnapTransform().containsNaN())
         {
            return snapData;
         }
         else if (computeWiggleTransform)
         {
            computeWiggleTransform(footstep, stanceStep, snapData);
         }

         return snapData;
      }
   }

   /**
    * Can manually add snap data for a footstep to bypass the snapper.
    */
   public void addSnapData(DiscreteFootstep footstep, FootstepSnapData snapData)
   {
      snapDataHolder.put(footstep, snapData);
   }

   protected FootstepSnapData computeSnapTransform(DiscreteFootstep footstepToSnap, DiscreteFootstep stanceStep)
   {
      double maximumRegionHeightToConsider = getMaximumRegionHeightToConsider(stanceStep);
      DiscreteFootstepTools.getFootPolygon(footstepToSnap, footPolygonsInSoleFrame.get(footstepToSnap.getRobotSide()), footPolygon);

      RigidBodyTransform snapTransform;

      if (heightMapData == null)
      {
         snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon, planarRegionsList, maximumRegionHeightToConsider, planarRegionToPack);
      }
      else
      {
         snapTransform = heightMapSnapper.snapPolygonToHeightMap(footPolygon, heightMapData, parameters.getHeightMapSnapThreshold());
      }

      if (snapTransform == null)
      {
         return FootstepSnapData.emptyData();
      }
      else
      {
         FootstepSnapData snapData = new FootstepSnapData(snapTransform);

         if (planarRegionsList != null)
         {
            snapData.setRegionIndex(getIndex(planarRegionToPack, planarRegionsList));
            computeCroppedFoothold(footstepToSnap, snapData);
         }
         if (heightMapData != null)
         {
            snapData.getWiggleTransformInWorld().setIdentity();
            snapData.setRMSErrorHeightMap(heightMapSnapper.getRMSError());
            snapData.setHeightMapArea(heightMapSnapper.getArea());
         }

         return snapData;
      }
   }

   private static int getIndex(PlanarRegion planarRegion, PlanarRegionsList planarRegionsList)
   {
      double epsilon = 1e-7;
      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         if (planarRegionsList.getPlanarRegion(i).epsilonEquals(planarRegion, epsilon))
         {
            return i;
         }
      }

      return -1;
   }

   private double getMaximumRegionHeightToConsider(DiscreteFootstep stanceStep)
   {
      if (stanceStep == null)
      {
         return Double.POSITIVE_INFINITY;
      }

      FootstepSnapData snapData = snapDataHolder.get(stanceStep);
      if (snapData == null || snapData.getSnapTransform().containsNaN())
      {
         return Double.POSITIVE_INFINITY;
      }
      else
      {
         return parameters.getMaximumSnapHeight() + DiscreteFootstepTools.getSnappedStepHeight(stanceStep, snapData.getSnapTransform());
      }
   }

   protected void computeWiggleTransform(DiscreteFootstep footstepToWiggle, DiscreteFootstep stanceStep, FootstepSnapData snapData)
   {
      int regionIndex = snapData.getRegionIndex();
      if (regionIndex == -1)
      {
         LogTools.warn("Could not find matching region id, unable to find wiggle transform. Region id = " + snapData.getRegionIndex());
         snapData.getWiggleTransformInWorld().setIdentity();
         return;
      }
      else
      {
         planarRegionToPack.set(planarRegionsList.getPlanarRegion(regionIndex));
      }

      DiscreteFootstepTools.getFootPolygon(footstepToWiggle, footPolygonsInSoleFrame.get(footstepToWiggle.getRobotSide()), footPolygon);
      tempTransform.set(snapData.getSnapTransform());
      tempTransform.preMultiply(planarRegionToPack.getTransformToLocal());
      ConvexPolygon2D footPolygonInRegionFrame = FootstepSnappingTools.computeTransformedPolygon(footPolygon, tempTransform);

      RigidBodyTransform wiggleTransformInLocal;
      boolean concaveWigglerRequested = parameters.getEnableConcaveHullWiggler() && !planarRegionToPack.getConcaveHull().isEmpty();
      if (concaveWigglerRequested)
      {
         gradientDescentStepConstraintInput.clear();
         gradientDescentStepConstraintInput.setInitialStepPolygon(footPolygonInRegionFrame);
         gradientDescentStepConstraintInput.setWiggleParameters(wiggleParameters);
         gradientDescentStepConstraintInput.setPlanarRegion(planarRegionToPack);

         if (stanceStep != null && snapDataHolder.containsKey(stanceStep))
         {
            FootstepSnapData stanceSnapData = snapDataHolder.get(stanceStep);
            DiscreteFootstepTools.getFootPolygon(stanceStep, footPolygonsInSoleFrame.get(stanceStep.getRobotSide()), footPolygon);

            DiscreteFootstepTools.getFootPolygon(stanceStep, footPolygonsInSoleFrame.get(stanceStep.getRobotSide()), footPolygon);
            ConvexPolygon2D stancePolygonInStanceSnapFrame = FootstepSnappingTools.computeTransformedPolygon(footPolygon, stanceSnapData.getSnapTransform());

            // transform from stance snap frame to region local
            tempTransform.set(stanceSnapData.getSnapTransform());

            int stanceRegionIndex = stanceSnapData.getRegionIndex();
            if (stanceRegionIndex != -1)
            {
               PlanarRegion stanceStepRegion = planarRegionsList.getPlanarRegion(stanceRegionIndex);
               tempTransform.preMultiply(stanceStepRegion.getTransformToLocal());
               tempTransform.preMultiply(stanceSnapData.getWiggleTransformInWorld());
               tempTransform.preMultiply(stanceStepRegion.getTransformToWorld());
            }

            tempTransform.preMultiply(planarRegionToPack.getTransformToLocal());

            Point3D stanceVertex3D = new Point3D();
            ConvexPolygon2D projectedStanceFoot = new ConvexPolygon2D();

            for (int i = 0; i < stancePolygonInStanceSnapFrame.getNumberOfVertices(); i++)
            {
               stanceVertex3D.set(footPolygon.getVertex(i).getX(), footPolygon.getVertex(i).getY(), 0.0);
               tempTransform.transform(stanceVertex3D);

               // project to xy in region local
               projectedStanceFoot.addVertex(stanceVertex3D.getX(), stanceVertex3D.getY());
            }

            projectedStanceFoot.update();
            gradientDescentStepConstraintInput.setStanceFootPolygon(projectedStanceFoot);
            gradientDescentStepConstraintSolver.setStanceFootClearance(parameters.getMinClearanceFromStance());
         }

         if (parameters.getEnableShinCollisionCheck())
         {
            RigidBodyTransform snappedStepTransform = snapData.getSnappedStepTransform(footstepToWiggle);
            tempTransform.set(snappedStepTransform);
            tempTransform.preMultiply(planarRegionToPack.getTransformToLocal());
            gradientDescentStepConstraintInput.setFootstepInRegionFrame(tempTransform);

            ConvexPolygon2D footPolygon = footPolygonsInSoleFrame.get(footstepToWiggle.getRobotSide());
            double forwardPoint = footPolygon.getMaxX() + parameters.getShinToeClearance();
            double backwardPoint = footPolygon.getMinX() - parameters.getShinHeelClearance();
            double shinRadius = 0.5 * (forwardPoint - backwardPoint);
            double shinXOffset = 0.5 * (forwardPoint + backwardPoint);

            legCollisionShape.setSize(parameters.getShinLength(), shinRadius);
            transformGenerator.identity();
            transformGenerator.translate(shinXOffset, 0.0, parameters.getShinHeightOffset());
            transformGenerator.translate(0.0, 0.0, 0.5 * parameters.getShinLength());
            transformGenerator.getRigidyBodyTransform(legCollisionShapeToSoleTransform);
            gradientDescentStepConstraintSolver.setLegCollisionShape(legCollisionShape, legCollisionShapeToSoleTransform);

            gradientDescentStepConstraintInput.setPlanarRegionsList(planarRegionsList);
         }

         wiggleTransformInLocal = gradientDescentStepConstraintSolver.wigglePolygon(gradientDescentStepConstraintInput);
      }
      else
      {
         double initialDeltaInside = computeAchievedDeltaInside(footPolygonInRegionFrame, planarRegionToPack, false);
         if (initialDeltaInside > parameters.getWiggleInsideDeltaTarget())
         {
            snapData.setAchievedInsideDelta(initialDeltaInside);
            snapData.getWiggleTransformInWorld().setIdentity();
            return;
         }
         else
         {
            if ((wiggleTransformInLocal = wiggleIntoConvexHull(footPolygonInRegionFrame)) == null)
            {
               snapData.getWiggleTransformInWorld().setIdentity();
               return;
            }
         }
      }

      // compute achieved delta inside
      footPolygonInRegionFrame.applyTransform(wiggleTransformInLocal, false);
      snapData.setAchievedInsideDelta(computeAchievedDeltaInside(footPolygonInRegionFrame, planarRegionToPack, concaveWigglerRequested));

      // compute wiggle transform in world
      snapData.getWiggleTransformInWorld().set(planarRegionToPack.getTransformToLocal());
      snapData.getWiggleTransformInWorld().preMultiply(wiggleTransformInLocal);
      snapData.getWiggleTransformInWorld().preMultiply(planarRegionToPack.getTransformToWorld());

      if (!parameters.getEnableConcaveHullWiggler() && stanceStep != null && snapDataHolder.containsKey(stanceStep))
      {
         FootstepSnapData stanceStepSnapData = snapDataHolder.get(stanceStep);

         // check for overlap
         boolean overlapDetected = stepsAreTooClose(footstepToWiggle, snapData, stanceStep, stanceStepSnapData);
         if (overlapDetected)
         {
            snapData.getWiggleTransformInWorld().setIdentity();
         }

         // check for overlap after this steps wiggle is removed. if still overlapping, remove wiggle on stance step
         overlapDetected = stepsAreTooClose(footstepToWiggle, snapData, stanceStep, stanceStepSnapData);
         if (overlapDetected)
         {
            stanceStepSnapData.getWiggleTransformInWorld().setIdentity();
         }
      }

      computeCroppedFoothold(footstepToWiggle, snapData);
   }

   /** Extracted to method for testing purposes */
   protected RigidBodyTransform wiggleIntoConvexHull(ConvexPolygon2D footPolygonInRegionFrame)
   {
      return PolygonWiggler.wigglePolygonIntoConvexHullOfRegion(footPolygonInRegionFrame, planarRegionToPack, wiggleParameters);
   }

   private final RigidBodyTransform transform1 = new RigidBodyTransform();
   private final RigidBodyTransform transform2 = new RigidBodyTransform();
   private final ConvexPolygon2D polygon1 = new ConvexPolygon2D();
   private final ConvexPolygon2D polygon2 = new ConvexPolygon2D();

   /** Extracted to method for testing purposes */
   protected boolean stepsAreTooClose(DiscreteFootstep step1, FootstepSnapData snapData1, DiscreteFootstep step2, FootstepSnapData snapData2)
   {
      DiscreteFootstepTools.getFootPolygon(step1, footPolygonsInSoleFrame.get(step1.getRobotSide()), polygon1);
      DiscreteFootstepTools.getFootPolygon(step2, footPolygonsInSoleFrame.get(step2.getRobotSide()), polygon2);

      snapData1.packSnapAndWiggleTransform(transform1);
      snapData2.packSnapAndWiggleTransform(transform2);

      polygon1.applyTransform(transform1, false);
      polygon2.applyTransform(transform2, false);

      boolean intersection = StepConstraintPolygonTools.arePolygonsIntersecting(polygon1, polygon2);
      if (intersection)
      {
         return true;
      }

      double distance = StepConstraintPolygonTools.distanceBetweenPolygons(polygon1, polygon2);
      return distance < parameters.getMinClearanceFromStance();
   }

   protected void computeCroppedFoothold(DiscreteFootstep footstep, FootstepSnapData snapData)
   {
      if (flatGroundMode())
      {
         snapData.getCroppedFoothold().clearAndUpdate();
         return;
      }

      DiscreteFootstepTools.getFootPolygon(footstep, footPolygonsInSoleFrame.get(footstep.getRobotSide()), footPolygon);

      snapData.packSnapAndWiggleTransform(tempTransform);
      ConvexPolygon2D snappedPolygonInWorld = FootstepSnappingTools.computeTransformedPolygon(footPolygon, tempTransform);
      ConvexPolygon2D croppedFootPolygon = FootstepSnappingTools.computeRegionIntersection(planarRegionToPack, snappedPolygonInWorld);

      if (!croppedFootPolygon.isEmpty())
      {
         FootstepSnappingTools.changeFromPlanarRegionToSoleFrame(planarRegionToPack, footstep, tempTransform, croppedFootPolygon);
         snapData.getCroppedFoothold().set(croppedFootPolygon);
      }
   }

   private static void updateWiggleParameters(WiggleParameters wiggleParameters, FootstepPlannerParametersReadOnly parameters)
   {
      wiggleParameters.deltaInside = parameters.getWiggleInsideDeltaTarget();
      wiggleParameters.maxX = parameters.getMaximumXYWiggleDistance();
      wiggleParameters.minX = -parameters.getMaximumXYWiggleDistance();
      wiggleParameters.maxY = parameters.getMaximumXYWiggleDistance();
      wiggleParameters.minY = -parameters.getMaximumXYWiggleDistance();
      wiggleParameters.maxYaw = parameters.getMaximumYawWiggle();
      wiggleParameters.minYaw = -parameters.getMaximumYawWiggle();
   }

   /** package-private for testing */
   static double computeAchievedDeltaInside(ConvexPolygon2DReadOnly footPolygon, PlanarRegion planarRegion, boolean useConcaveHull)
   {
      double achievedDeltaInside = Double.POSITIVE_INFINITY;
      ToDoubleFunction<Point2DReadOnly> deltaInsideCalculator;

      if (useConcaveHull)
      {
         Vertex2DSupplier concaveHullVertices = Vertex2DSupplier.asVertex2DSupplier(planarRegion.getConcaveHull());
         deltaInsideCalculator = vertex ->
         {
            boolean pointIsInside = StepConstraintPolygonTools.isPointInsidePolygon(concaveHullVertices, vertex);
            double distanceSquaredFromPerimeter = FootPlacementConstraintCalculator.distanceSquaredFromPerimeter(concaveHullVertices, vertex, null);
            return (pointIsInside ? 1 : -1) * Math.sqrt(distanceSquaredFromPerimeter);
         };
      }
      else
      {
         deltaInsideCalculator = vertex -> - planarRegion.getConvexHull().signedDistance(vertex);
      }

      for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
      {
         double insideDelta = deltaInsideCalculator.applyAsDouble(footPolygon.getVertex(i));
         if (insideDelta < achievedDeltaInside)
         {
            achievedDeltaInside = insideDelta;
         }
      }

      return achievedDeltaInside;
   }

   /**
    * Clears snapper history
    */
   public void reset()
   {
      snapDataHolder.clear();
   }
}
