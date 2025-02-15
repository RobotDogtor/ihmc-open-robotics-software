package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class OneStepCaptureRegionCalculator
{
   private final CaptureRegionMathTools captureRegionMath = new CaptureRegionMathTools();

   private static final boolean VISUALIZE = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int MAX_CAPTURE_REGION_POLYGON_POINTS = 20;
   private static final int KINEMATIC_LIMIT_POINTS = 8;
   private double reachableRegionCutoffAngle = 1.0;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final ExecutionTimer globalTimer = new ExecutionTimer(name + "Timer", registry);

   private CaptureRegionVisualizer captureRegionVisualizer = null;
   private final FrameConvexPolygon2D captureRegionPolygon = new FrameConvexPolygon2D(worldFrame);

   // necessary variables for the reachable region and capture calculation:
//   private final double midFootAnkleXOffset;
   private final double footWidth;
   private final DoubleProvider kinematicStepRange;
   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;
   private final SideDependentList<FrameConvexPolygon2D> reachableRegions = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());

   private final RecyclingArrayList<FramePoint2D> visibleVertices = new RecyclingArrayList<>(MAX_CAPTURE_REGION_POLYGON_POINTS, FramePoint2D.class);

   private boolean haveReachableRegionsBeenCalculated = false;

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   private final boolean useInternalReachableRegions;

   public OneStepCaptureRegionCalculator(CommonHumanoidReferenceFrames referenceFrames, WalkingControllerParameters walkingControllerParameters,
         YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters.getSteppingParameters().getFootWidth(),
           () -> walkingControllerParameters.getSteppingParameters().getMaxStepLength(),
           referenceFrames.getSoleZUpFrames(),
           true,
           "",
           parentRegistry,
           yoGraphicsListRegistry);
   }


   public OneStepCaptureRegionCalculator(SideDependentList<? extends ReferenceFrame> soleZUpFrames, WalkingControllerParameters walkingControllerParameters,
                                         String suffix, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters.getSteppingParameters().getFootWidth(),
           () -> walkingControllerParameters.getSteppingParameters().getMaxStepLength(),
           soleZUpFrames,
           true,
           suffix,
           parentRegistry,
           yoGraphicsListRegistry);
   }

   public OneStepCaptureRegionCalculator(SideDependentList<? extends ReferenceFrame> soleZUpFrames, WalkingControllerParameters walkingControllerParameters,
                                         boolean useInternalReachableRegions,
                                         String suffix, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters.getSteppingParameters().getFootWidth(),
           () -> walkingControllerParameters.getSteppingParameters().getMaxStepLength(),
           soleZUpFrames,
           useInternalReachableRegions,
           suffix,
           parentRegistry,
           yoGraphicsListRegistry);
   }


   public OneStepCaptureRegionCalculator(double footWidth, double kinematicStepRange,
                                         SideDependentList<? extends ReferenceFrame> soleZUpFrames, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(footWidth, () -> kinematicStepRange, soleZUpFrames, true, "", parentRegistry, yoGraphicsListRegistry);
   }

   public OneStepCaptureRegionCalculator(double footWidth,
                                         DoubleProvider kinematicStepRange,
                                         SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                         boolean useInternalReachableRegions,
                                         String suffix, YoRegistry parentRegistry,
                                         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.kinematicStepRange = kinematicStepRange;
      this.soleZUpFrames = soleZUpFrames;
      this.useInternalReachableRegions = useInternalReachableRegions;
//      this.midFootAnkleXOffset = midFootAnkleXOffset;
      this.footWidth = footWidth;

      // set up registry and visualizer
      parentRegistry.addChild(registry);
      if (yoGraphicsListRegistry != null && VISUALIZE)
      {
         captureRegionVisualizer = new CaptureRegionVisualizer(this::getCaptureRegion, suffix, yoGraphicsListRegistry, registry);
      }
   }

   private void calculateReachableRegions(double footWidth)
   {
      for (RobotSide side : RobotSide.values)
      {
         FrameConvexPolygon2D reachableRegion = reachableRegions.get(side);
         reachableRegion.clear(soleZUpFrames.get(side));
         double sign = side.negateIfLeftSide(1.0);

         for (int i = 0; i < MAX_CAPTURE_REGION_POLYGON_POINTS - 1; i++)
         {
            double angle = sign * reachableRegionCutoffAngle * Math.PI * (i) / (MAX_CAPTURE_REGION_POLYGON_POINTS - 2);
            double x = kinematicStepRange.getValue() * Math.cos(angle);
            double y = kinematicStepRange.getValue() * Math.sin(angle);
            if (Math.abs(y) < footWidth / 2.0)
               y = sign * footWidth / 2.0;
            reachableRegion.addVertex(soleZUpFrames.get(side), x, y);
         }
         reachableRegion.addVertex(soleZUpFrames.get(side), 0, sign * footWidth / 2.0);
         reachableRegion.update();
      }
   }

   // variables for the capture region calculation
   private static final int APPROXIMATION_MULTIPLIER = 1;
   private final FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D();
   private final FramePoint2D footCentroid = new FramePoint2D(worldFrame);
   private final FramePoint2D predictedICP = new FramePoint2D(worldFrame);
   private final FramePoint2D capturePoint = new FramePoint2D(worldFrame);
   private final FramePoint2D kinematicExtreme = new FramePoint2D(worldFrame);
   private final FramePoint2D additionalKinematicPoint = new FramePoint2D(worldFrame);
   private final FrameVector2D firstKinematicExtremeDirection = new FrameVector2D(worldFrame);
   private final FrameVector2D lastKinematicExtremeDirection = new FrameVector2D(worldFrame);
   private final FrameConvexPolygon2D rawCaptureRegion = new FrameConvexPolygon2D(worldFrame);

   public void calculateCaptureRegion(RobotSide swingSide, double swingTimeRemaining, FramePoint2DReadOnly icp, double omega0, FrameConvexPolygon2DReadOnly footPolygon)
   {
      globalTimer.startMeasurement();

      if (!haveReachableRegionsBeenCalculated)
      {
         calculateReachableRegions(footWidth);
         haveReachableRegionsBeenCalculated = true;
      }

      // 1. Set up all needed variables and reference frames for the calculation:
      ReferenceFrame supportSoleZUp = soleZUpFrames.get(swingSide.getOppositeSide());
         // change the support foot polygon only if the swing side changed to avoid garbage every tick.
      this.supportFootPolygon.setIncludingFrame(footPolygon);
      this.supportFootPolygon.changeFrameAndProjectToXYPlane(supportSoleZUp);

      capturePoint.setIncludingFrame(icp);
      capturePoint.changeFrame(supportSoleZUp);
      firstKinematicExtremeDirection.setToZero(supportSoleZUp);
      lastKinematicExtremeDirection.setToZero(supportSoleZUp);
      predictedICP.setToZero(supportSoleZUp);

      swingTimeRemaining = MathTools.clamp(swingTimeRemaining, 0.0, Double.POSITIVE_INFINITY);
      footCentroid.setIncludingFrame(supportFootPolygon.getCentroid());
      rawCaptureRegion.clear(supportSoleZUp);
      captureRegionPolygon.clear(supportSoleZUp);
      kinematicExtreme.setToZero(supportSoleZUp);

      // 2. Get extreme CoP positions
      boolean icpOutsideSupport = computeVisibleVerticesFromOutsideLeftToRightCopy(supportFootPolygon, capturePoint);
      FrameConvexPolygon2D reachableRegion = reachableRegions.get(swingSide.getOppositeSide());
      if (!icpOutsideSupport)
      {
         // If the ICP is in the support polygon return the whole reachable region.
         globalTimer.stopMeasurement();
         captureRegionPolygon.setIncludingFrame(reachableRegion);
         updateVisualizer();
         return;
      }

      // 3. For every possible extreme CoP predict the corresponding ICP given the remaining swing time
      int lastIndex = visibleVertices.size() - 1;
      for (int i = 0; i < visibleVertices.size(); i++)
      {
         FramePoint2D copExtreme = visibleVertices.get(i);
         CapturePointTools.computeDesiredCapturePointPosition(omega0, swingTimeRemaining, capturePoint, copExtreme, predictedICP);
         rawCaptureRegion.addVertexMatchingFrame(predictedICP, false);

         // 4. Project the predicted ICP on a circle around the foot with the radius of the step range.
         int intersections = EuclidCoreMissingTools.intersectionBetweenRay2DAndCircle(APPROXIMATION_MULTIPLIER * kinematicStepRange.getValue(), footCentroid, copExtreme,
                                                                                      predictedICP, kinematicExtreme, null);
         if (intersections > 1)
            throw new RuntimeException("The cop was outside of the reachable range.");

         if (kinematicExtreme.containsNaN())
         {
            globalTimer.stopMeasurement();
            captureRegionPolygon.update();
            return;
         }
         rawCaptureRegion.addVertexMatchingFrame(kinematicExtreme, false);

         if (i == 0)
            firstKinematicExtremeDirection.sub(kinematicExtreme, footCentroid);
         else if (i == lastIndex)
            lastKinematicExtremeDirection.sub(kinematicExtreme, footCentroid);
      }

      // 5. Add additional points to the capture region polygon on the circle between the kinematic extreme points
      for (int i = 0; i < KINEMATIC_LIMIT_POINTS - 1; i++)
      {
         double alphaFromAToB = ((double) (i + 1)) / ((double) (KINEMATIC_LIMIT_POINTS + 1));
         captureRegionMath.getPointBetweenVectorsAtDistanceFromOriginCircular(firstKinematicExtremeDirection, lastKinematicExtremeDirection, alphaFromAToB,
                                                                              APPROXIMATION_MULTIPLIER * kinematicStepRange.getValue(), footCentroid, additionalKinematicPoint);
         rawCaptureRegion.addVertexMatchingFrame(additionalKinematicPoint, false);
      }

      // 6. Intersect the capture region with the reachable region
      if (!rawCaptureRegion.isEmpty())
      {
         rawCaptureRegion.update();

         if (useInternalReachableRegions)
         {
            // This causes the capture region to always be null if it is null once.
            // This assumes that once there is no capture region the robot will fall for sure.
            rawCaptureRegion.checkReferenceFrameMatch(reachableRegion);
            captureRegionPolygon.clear(rawCaptureRegion.getReferenceFrame());
            convexPolygonTools.computeIntersectionOfPolygons(rawCaptureRegion, reachableRegion, captureRegionPolygon);
         }
         else
         {
            captureRegionPolygon.setIncludingFrame(rawCaptureRegion);
         }
      }

      captureRegionPolygon.update();

      globalTimer.stopMeasurement();
      updateVisualizer();
   }

   private void updateVisualizer()
   {
      if (captureRegionVisualizer != null)
      {
         captureRegionVisualizer.update();
      }
      else
      {
         hideCaptureRegion();
      }
   }

   public void hideCaptureRegion()
   {
      if (captureRegionVisualizer != null)
      {
         captureRegionVisualizer.hide();
      }
   }

   public FrameConvexPolygon2D getCaptureRegion()
   {
      return captureRegionPolygon;
   }

   public void setReachableRegionCutoffAngle(double reachableRegionCutoffAngle)
   {
      this.reachableRegionCutoffAngle = reachableRegionCutoffAngle;
      calculateReachableRegions(footWidth);
   }

   public FrameConvexPolygon2D getReachableRegion(RobotSide robotSide)
   {
      return reachableRegions.get(robotSide);
   }

   public double getKinematicStepRange()
   {
      return kinematicStepRange.getValue();
   }

   public double getCaptureRegionArea()
   {
      captureRegionPolygon.update();
      return captureRegionPolygon.getArea();
   }

   /**
    * Returns all of the vertices that are visible from the observerPoint2d, in left to right order.
    * If the observerPoint2d is inside the polygon, returns null.
    *
    * @param observerFramePoint Point2d
    * @return Point2d[]
    */
   public boolean computeVisibleVerticesFromOutsideLeftToRightCopy(FrameConvexPolygon2DReadOnly convexPolygon, FramePoint2DReadOnly observerFramePoint)
   {
      visibleVertices.clear();

      convexPolygon.checkReferenceFrameMatch(observerFramePoint);
      int lineOfSightStartIndex = convexPolygon.lineOfSightStartIndex(observerFramePoint);
      int lineOfSightEndIndex = convexPolygon.lineOfSightEndIndex(observerFramePoint);
      if (lineOfSightStartIndex == -1 || lineOfSightEndIndex == -1)
         return false;

      int index = lineOfSightEndIndex;

      while (true)
      {
         visibleVertices.add().setIncludingFrame(convexPolygon.getVertex(index));
         index = convexPolygon.getPreviousVertexIndex(index);
         if (index == lineOfSightStartIndex)
         {
            visibleVertices.add().setIncludingFrame(convexPolygon.getVertex(index));
            break;
         }
      }

      return true;
   }
}
