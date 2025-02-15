package us.ihmc.gdx.vr;

import com.badlogic.gdx.utils.BufferUtils;
import org.lwjgl.openvr.*;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

import java.nio.LongBuffer;
import java.util.function.Consumer;

public class GDXVRHeadset extends GDXVRTrackedDevice
{
   private final LongBuffer inputSourceHandle = BufferUtils.newLongBuffer(1);
   private final RigidBodyTransform headsetToWorldTransform = new RigidBodyTransform();
   private static final RigidBodyTransformReadOnly headsetXRightZDownToXForwardZUp = new RigidBodyTransform(
         new YawPitchRoll(         // For this transformation, we start with IHMC ZUp with index forward and thumb up
            Math.toRadians(90.0),  // rotating around thumb, index goes forward to left
            Math.toRadians(90.0),  // rotating about middle finger, index goes left to down
            Math.toRadians(0.0)    // no rotation about index finger
         ),
         new Point3D()
   );
   private final ReferenceFrame xForwardZUpHeadsetFrame;

   public GDXVRHeadset(ReferenceFrame vrPlayAreaYUpZBackFrame)
   {
      super(vrPlayAreaYUpZBackFrame);
      setDeviceIndex(VR.k_unTrackedDeviceIndex_Hmd);

      xForwardZUpHeadsetFrame
            = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("xForwardZUpHeadsetFrame",
                                                                                getDeviceYUpZBackFrame(),
                                                                                headsetXRightZDownToXForwardZUp);
   }

   public void initSystem()
   {
      VRInput.VRInput_GetInputSourceHandle("/user/head", inputSourceHandle);
   }

   public void update(TrackedDevicePose.Buffer trackedDevicePoses)
   {
      setConnected(VRSystem.VRSystem_IsTrackedDeviceConnected(VR.k_unTrackedDeviceIndex_Hmd));

      super.update(trackedDevicePoses);
   }

   public void runIfConnected(Consumer<GDXVRHeadset> runIfConnected)
   {
      if (isConnected())
      {
         runIfConnected.accept(this);
      }
   }

   public ReferenceFrame getXForwardZUpHeadsetFrame()
   {
      return xForwardZUpHeadsetFrame;
   }
}
