package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalProvider;

public class CoMTrajectorySegment implements FixedFramePositionTrajectoryGenerator, TimeIntervalProvider, Settable<CoMTrajectorySegment>
{
   private final FramePoint3D firstCoefficient = new FramePoint3D();
   private final FramePoint3D secondCoefficient = new FramePoint3D();
   private final FramePoint3D thirdCoefficient = new FramePoint3D();
   private final FramePoint3D fourthCoefficient = new FramePoint3D();
   private final FramePoint3D fifthCoefficient = new FramePoint3D();
   private final FramePoint3D sixthCoefficient = new FramePoint3D();

   private final FramePoint3D comPosition = new FramePoint3D();
   private final FrameVector3D comVelocity = new FrameVector3D();
   private final FrameVector3D comAcceleration = new FrameVector3D();

   private final FramePoint3D dcmPosition = new FramePoint3D();
   private final FrameVector3D dcmVelocity = new FrameVector3D();
   private final FramePoint3D vrpPosition = new FramePoint3D();
   private final FrameVector3D vrpVelocity = new FrameVector3D();

   private double currentTime;
   private double omega = 3.0;
   private final TimeIntervalBasics timeInterval = new TimeInterval();

   public void reset()
   {
      currentTime = Double.NaN;
      firstCoefficient.setToNaN();
      secondCoefficient.setToNaN();
      thirdCoefficient.setToNaN();
      fourthCoefficient.setToNaN();
      fifthCoefficient.setToNaN();
      sixthCoefficient.setToNaN();

      timeInterval.reset();
   }

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }

   @Override
   public void set(CoMTrajectorySegment other)
   {
      getTimeInterval().set(other.getTimeInterval());
      currentTime = other.currentTime;
      omega = other.omega;
      setCoefficients(other);

      comPosition.set(other.comPosition);
      comVelocity.set(other.comVelocity);
      comAcceleration.set(other.comAcceleration);

      dcmPosition.set(other.dcmPosition);
      dcmVelocity.set(other.dcmVelocity);
      vrpPosition.set(other.vrpPosition);
      vrpVelocity.set(other.vrpVelocity);
   }

   public void setCoefficients(DMatrixRMaj coefficients)
   {
      setCoefficients(coefficients, 0);
   }

   public void setCoefficients(DMatrixRMaj coefficients, int startRow)
   {
      setFirstCoefficient(ReferenceFrame.getWorldFrame(), coefficients.get(startRow, 0), coefficients.get(startRow, 1), coefficients.get(startRow, 2));
      setSecondCoefficient(ReferenceFrame.getWorldFrame(), coefficients.get(startRow + 1, 0), coefficients.get(startRow + 1, 1), coefficients.get(startRow + 1, 2));
      setThirdCoefficient(ReferenceFrame.getWorldFrame(), coefficients.get(startRow + 2, 0), coefficients.get(startRow + 2, 1), coefficients.get(startRow + 2, 2));
      setFourthCoefficient(ReferenceFrame.getWorldFrame(), coefficients.get(startRow + 3, 0), coefficients.get(startRow + 3, 1), coefficients.get(startRow + 3, 2));
      setFifthCoefficient(ReferenceFrame.getWorldFrame(), coefficients.get(startRow + 4, 0), coefficients.get(startRow + 4, 1), coefficients.get(startRow + 4, 2));
      setSixthCoefficient(ReferenceFrame.getWorldFrame(), coefficients.get(startRow + 5, 0), coefficients.get(startRow + 5, 1), coefficients.get(startRow + 5, 2));
   }

   public void setCoefficients(CoMTrajectorySegment other)
   {
      setCoefficients(other.firstCoefficient,
                      other.secondCoefficient,
                      other.thirdCoefficient,
                      other.fourthCoefficient,
                      other.fifthCoefficient,
                      other.sixthCoefficient);
   }

   public void setCoefficients(FramePoint3DReadOnly firstCoefficient,
                               FramePoint3DReadOnly secondCoefficient,
                               FramePoint3DReadOnly thirdCoefficient,
                               FramePoint3DReadOnly fourthCoefficient,
                               FramePoint3DReadOnly fifthCoefficient,
                               FramePoint3DReadOnly sixthCoefficient)
   {
      setFirstCoefficient(firstCoefficient);
      setSecondCoefficient(secondCoefficient);
      setThirdCoefficient(thirdCoefficient);
      setFourthCoefficient(fourthCoefficient);
      setFifthCoefficient(fifthCoefficient);
      setSixthCoefficient(sixthCoefficient);
   }

   public void setFirstCoefficient(FramePoint3DReadOnly firstCoefficient)
   {
      setFirstCoefficient(firstCoefficient.getReferenceFrame(), firstCoefficient.getX(), firstCoefficient.getY(), firstCoefficient.getZ());
   }

   public void setFirstCoefficient(ReferenceFrame frame, double x, double y, double z)
   {
      firstCoefficient.set(frame, x, y, z);
   }

   public void setSecondCoefficient(FramePoint3DReadOnly secondCoefficient)
   {
      this.secondCoefficient.set(secondCoefficient);
   }

   public void setSecondCoefficient(ReferenceFrame frame, double x, double y, double z)
   {
      secondCoefficient.set(frame, x, y, z);
   }

   public void setThirdCoefficient(FramePoint3DReadOnly thirdCoefficient)
   {
      this.thirdCoefficient.set(thirdCoefficient);
   }

   public void setThirdCoefficient(ReferenceFrame frame, double x, double y, double z)
   {
      thirdCoefficient.set(frame, x, y, z);
   }

   public void setFourthCoefficient(FramePoint3DReadOnly fourthCoefficient)
   {
      this.fourthCoefficient.set(fourthCoefficient);
   }

   public void setFourthCoefficient(ReferenceFrame frame, double x, double y, double z)
   {
      fourthCoefficient.set(frame, x, y, z);
   }

   public void setFifthCoefficient(FramePoint3DReadOnly fifthCoefficient)
   {
      this.fifthCoefficient.set(fifthCoefficient);
   }

   public void setFifthCoefficient(ReferenceFrame frame, double x, double y, double z)
   {
      fifthCoefficient.set(frame, x, y, z);
   }

   public void setSixthCoefficient(FramePoint3DReadOnly sixthCoefficient)
   {
      this.sixthCoefficient.set(sixthCoefficient);
   }

   public void setSixthCoefficient(ReferenceFrame frame, double x, double y, double z)
   {
      sixthCoefficient.set(frame, x, y, z);
   }

   private final FramePoint3D modifiedFourthCoefficient = new FramePoint3D();
   private final FramePoint3D modifiedFifthCoefficient = new FramePoint3D();
   private final FramePoint3D modifiedSixthCoefficient = new FramePoint3D();

   public void shiftStartOfSegment(double durationToShift)
   {
      double originalDuration = getTimeInterval().getDuration();
      if (durationToShift > originalDuration)
         throw new IllegalArgumentException("New start time " + durationToShift + " must be less than end time " + originalDuration);

      double d2 = durationToShift * durationToShift;
      double d3 = d2 * durationToShift;
      double startTime = getTimeInterval().getStartTime();
      getTimeInterval().setInterval(startTime + durationToShift, getTimeInterval().getEndTime());
      double exponential = Math.exp(omega * durationToShift);
      firstCoefficient.scale(exponential);
      secondCoefficient.scale(1.0 / exponential);
      modifiedFourthCoefficient.scaleAdd(3.0 * durationToShift, thirdCoefficient, fourthCoefficient);
      modifiedFifthCoefficient.scaleAdd(3.0 * d2, thirdCoefficient, fifthCoefficient);
      modifiedFifthCoefficient.scaleAdd(2.0 * durationToShift, fourthCoefficient, modifiedFifthCoefficient);
      modifiedSixthCoefficient.scaleAdd(d3, thirdCoefficient, sixthCoefficient);
      modifiedSixthCoefficient.scaleAdd(d2, fourthCoefficient, modifiedSixthCoefficient);
      modifiedSixthCoefficient.scaleAdd(durationToShift, fifthCoefficient, modifiedSixthCoefficient);

      fourthCoefficient.set(modifiedFourthCoefficient);
      fifthCoefficient.set(modifiedFifthCoefficient);
      sixthCoefficient.set(modifiedSixthCoefficient);
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public void computeCoMPosition(double time, FixedFramePoint3DBasics comPositionToPack)
   {
      CoMTrajectoryPlannerTools.constructDesiredCoMPosition(comPositionToPack,
                                                            firstCoefficient,
                                                            secondCoefficient,
                                                            thirdCoefficient,
                                                            fourthCoefficient,
                                                            fifthCoefficient,
                                                            sixthCoefficient,
                                                            time,
                                                            omega);
   }

   public void computeCoMVelocity(double time, FixedFrameVector3DBasics comVelocityToPack)
   {
      CoMTrajectoryPlannerTools.constructDesiredCoMVelocity(comVelocityToPack,
                                                            firstCoefficient,
                                                            secondCoefficient,
                                                            thirdCoefficient,
                                                            fourthCoefficient,
                                                            fifthCoefficient,
                                                            sixthCoefficient,
                                                            time,
                                                            omega);
   }

   public void computeCoMAcceleration(double time, FixedFrameVector3DBasics comAccelerationToPack)
   {
      CoMTrajectoryPlannerTools.constructDesiredCoMAcceleration(comAccelerationToPack,
                                                                firstCoefficient,
                                                                secondCoefficient,
                                                                thirdCoefficient,
                                                                fourthCoefficient,
                                                                fifthCoefficient,
                                                                sixthCoefficient,
                                                                time,
                                                                omega);
   }

   public void computeVRPVelocity(double time, FixedFrameVector3DBasics vrpVelocityToPack)
   {
      CoMTrajectoryPlannerTools.constructDesiredVRPVelocity(vrpVelocityToPack,
                                                                firstCoefficient,
                                                                secondCoefficient,
                                                                thirdCoefficient,
                                                                fourthCoefficient,
                                                                fifthCoefficient,
                                                                sixthCoefficient,
                                                                time,
                                                                omega);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void compute(double time)
   {
      compute(time, comPosition, comVelocity, comAcceleration, dcmPosition, dcmVelocity, vrpPosition, vrpVelocity);
   }

   public void compute(double time,
                       FixedFramePoint3DBasics comPositionToPack,
                       FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack,
                       FixedFramePoint3DBasics dcmPositionToPack,
                       FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack,
                       FixedFrameVector3DBasics vrpVelocityTPack)
   {
      currentTime = time;
      computeCoMPosition(time, comPositionToPack);
      computeCoMVelocity(time, comVelocityToPack);
      computeCoMAcceleration(time, comAccelerationToPack);

      CoMTrajectoryPlannerTools.constructDesiredVRPVelocity(vrpVelocityTPack, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient,
                                                            sixthCoefficient, time, omega);


      CapturePointTools.computeCapturePointPosition(comPositionToPack, comVelocityToPack, omega, dcmPositionToPack);
      CapturePointTools.computeCapturePointVelocity(comVelocityToPack, comAccelerationToPack, omega, dcmVelocityToPack);
      CapturePointTools.computeCentroidalMomentumPivot(dcmPositionToPack, dcmVelocityToPack, omega, vrpPositionToPack);
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return comPosition;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return comVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return comAcceleration;
   }

   public FramePoint3DReadOnly getDCMPosition()
   {
      return dcmPosition;
   }

   public FrameVector3DReadOnly getDCMVelocity()
   {
      return dcmVelocity;
   }

   public FramePoint3DReadOnly getVRPPosition()
   {
      return vrpPosition;
   }

   public FrameVector3DReadOnly getVRPVelocity()
   {
      return vrpVelocity;
   }

   public FramePoint3DReadOnly getFirstCoefficient()
   {
      return firstCoefficient;
   }

   public FramePoint3DReadOnly getSecondCoefficient()
   {
      return secondCoefficient;
   }

   public FramePoint3DReadOnly getThirdCoefficient()
   {
      return thirdCoefficient;
   }

   public FramePoint3DReadOnly getFourthCoefficient()
   {
      return fourthCoefficient;
   }

   public FramePoint3DReadOnly getFifthCoefficient()
   {
      return fifthCoefficient;
   }

   public FramePoint3DReadOnly getSixthCoefficient()
   {
      return sixthCoefficient;
   }

   @Override
   public boolean isDone()
   {
      return currentTime >= getTimeInterval().getEndTime();
   }

   @Override
   public void hideVisualization()
   {}

   @Override
   public void showVisualization()
   {
   }
}
