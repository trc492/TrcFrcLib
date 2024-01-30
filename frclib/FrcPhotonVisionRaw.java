/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package TrcFrcLib.frclib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import TrcCommonLib.trclib.TrcPose3D;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionTargetInfo;

import org.opencv.core.Rect;

public class FrcPhotonVisionRaw extends FrcRemoteVisionProcessor
{
    /**
     * This class encapsulates info of the detected object. It extends TrcVisionTargetInfo.ObjectInfo that requires
     * it to provide methods to return the detected object info.
     */
    public static class DetectedObject implements TrcVisionTargetInfo.ObjectInfo
    {
        public final Object target;
        public final TrcPose3D pose;
        public final double area;
        public final double pitch;
        public final double yaw;
        public final double pixelX, pixelY;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param target specifies the detected object.
         * @param pose specifies the detected target pose.
         * @param area specifies the area of the detected target.
         * @param pitch specifies the pitch angle of the detected target.
         * @param yaw specifies the yaw angle of the detected target.
         * @param pixelX specifies the x screen coordinate in pixels of the detected target.
         * @param pixelY specifies the y screen coordinate in pixels of the detected target.
         */
        public DetectedObject(
            Object target, TrcPose3D pose, double area, double pitch, double yaw, double pixelX, double pixelY)
        {
            this.target = target;
            this.pose = pose;
            this.area = area;
            this.pitch = pitch;
            this.yaw = yaw;
            this.pixelX = pixelX;
            this.pixelY = pixelY;
        }   //DetectedObject

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @Override
        public String toString()
        {
            return
                "{Target=" + target +
                ",Pose=" + pose + 
                ",area=" + area +
                ",pitch=" + pitch +
                ",yaw=" + yaw +
                ",pixelX=" + pixelX +
                ",pixelY=" + pixelY + "}";
        }   //toString

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getObjectRect()
        {
            throw new UnsupportedOperationException("PhotonVision processor does not provide object rectangle.");
        }   //getObjectRect

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        @Override
        public double getObjectArea()
        {
            return area;
        }   //getObjectArea

        /**
         * This method returns the pose of the detected object relative to the camera.
         *
         * @return pose of the detected object relative to camera.
         */
        @Override
        public TrcPose3D getObjectPose()
        {
            return pose;
        }   //getObjectPose

        /**
         * This method returns the objects real world width.
         *
         * @return object real world width, null if not supported.
         */
        @Override
        public Double getObjectWidth()
        {
            return null;
        }   //getObjectWidth

        /**
         * This method returns the objects real world depth.
         *
         * @return object real world depth, null if not supported.
         */
        @Override
        public Double getObjectDepth()
        {
            return null;
        }   //getObjectDepth

    }   //class DetectedObject

    public enum LEDMode
    {
        AUTO(0), OFF(1), BLINK(2), ON(3);

        private int value;

        LEDMode(int value)
        {
            this.value = value;
        }

        public int getValue()
        {
            return value;
        }
    }   //enum LEDMode

    private final NetworkTable networkTable;
    private final NetworkTableEntry hasTarget;
    private final NetworkTableEntry targetArea;
    private final NetworkTableEntry targetPitch;
    private final NetworkTableEntry targetYaw;
    private final NetworkTableEntry targetPixelX;
    private final NetworkTableEntry targetPixelY;
    private final NetworkTableEntry targetPose;
    private final NetworkTableEntry ledMode;
    private final NetworkTableEntry pipeline;
    // private DoubleSupplier depthSupplier = () -> 0.0;

    // public FrcPhotonVisionProcessor(String instanceName, String depthInput, DoubleUnaryOperator depthApproximator)
    // {
    //     this(instanceName);
    //     setDepthApproximator(depthInput, depthApproximator);
    // }

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name (PhotonVision NetworkTable name).
     */
    public FrcPhotonVisionRaw(String instanceName)
    {
        super(instanceName);

        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        networkTable = ntInstance.getTable(instanceName);
        ntInstance.addConnectionListener(false, this::connectionListener);

        hasTarget = networkTable.getEntry("hasTarget");
        targetArea = networkTable.getEntry("targetArea");
        targetPitch = networkTable.getEntry("targetPitch");
        targetYaw = networkTable.getEntry("targetYaw");
        targetPixelX = networkTable.getEntry("targetPxielX");
        targetPixelY = networkTable.getEntry("targetPxielY");
        targetPose = networkTable.getEntry("targetPose");

        ledMode = networkTable.getEntry("ledMode");
        pipeline = networkTable.getEntry("pipeline");
    }   //FrcPhotonVisionRaw

    /**
     * This method is called when a network table event happened. It monitors the network table connection.
     *
     * @param event specifies the network table event.
     */
    private void connectionListener(NetworkTableEvent event)
    {
        tracer.traceInfo(
            instanceName,
            "NetworkTableEvent(remoteIp=" + event.connInfo.remote_ip + ", connected=" + event.is(Kind.kConnected) + ")");
    }   //connectionListener

    /**
     * This method returns the detected object.
     *
     * @return detected object.
     */
    public DetectedObject getDetectedObject()
    {
        DetectedObject detectedObj = null;

        if (targetDetected())
        {
            // TODO: figure out how to get AprilTag ID into target.
            detectedObj = new DetectedObject(
                null, getPose(), getArea(), getPitch(), getYaw(), getPixelPointX(), getPixelPointY());
            tracer.traceDebug(instanceName, "DetectedObj=" + detectedObj);
        }

        return detectedObj;
    }   //getDetectedObject

    // public double getTargetDepth()
    // {
    //     return targetDetected() ? depthSupplier.getAsDouble() : 0.0;
    // }

    // public void setDepthApproximator(String input, DoubleUnaryOperator depthApproximator)
    // {
    //     depthSupplier = () -> depthApproximator.applyAsDouble(networkTable.getEntry(input).getDouble(0.0));
    // }

    //
    // Methods for accessing NetworkTable entries.
    //

    /**
     * This method checks if a target is current detected.
     *
     * @return true if target is detected, false otherwise.
     */
    public boolean targetDetected()
    {
        return hasTarget.getBoolean(false);
    }   //targetDetected

    /**
     * This method returns the yaw angle of the detected target. It assumes targetDetected has been called first to
     * make sure there is valid data.
     *
     * @return yaw angle of detected target.
     */
    public double getYaw()
    {
        return targetYaw.getDouble(0.0);
    }   //getYaw

    /**
     * This method returns the pitch angle of the detected target. It assumes targetDetected has been called first to
     * make sure there is valid data.
     *
     * @return pitch angle of detected target.
     */
    public double getPitch()
    {
        return targetPitch.getDouble(0.0);
    }   //getPitch

    /**
     * This method returns the area of the detected target. It assumes targetDetected has been called first to
     * make sure there is valid data.
     *
     * @return area of detected target.
     */
    public double getArea()
    {
        return targetArea.getDouble(0.0);
    }   //getArea

    /**
     * This method returns the pixel point X of the detected target's center of mass. It assumes targetDetected has
     * been called first to make sure there is valid data.
     *
     * @return pixel point X of detected target's center of mass.
     */
    public double getPixelPointX()
    {
        return targetPixelX.getDouble(0.0);
    }   //getPixelPointX

    /**
     * This method returns the pixel point Y of the detected target's center of mass. It assumes targetDetected has
     * been called first to make sure there is valid data.
     *
     * @return pixel point Y of detected target's center of mass.
     */
    public double getPixelPointY()
    {
        return targetPixelY.getDouble(0.0);
    }   //getPixelPointY

    /**
     * This method returns the detected target pose. It assumes targetDetected has been called first to make sure
     * there is valid data.
     *
     * @return detected target pose.
     */
    public TrcPose3D getPose()
    {
        double[] pose = targetPose.getDoubleArray(new double[6]);
        return new TrcPose3D(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    }   //getPose

    /**
     * This method returns the selected PhotonVision pipeline.
     *
     * @return PhotonVision pipeline index.
     */
    public int getSelectedPipeline()
    {
        return (int) pipeline.getDouble(0.0);
    }   //getSelectedPipeline

    /**
     * This method selects the PhotonVision pipeline.
     *
     * @param pipeline specifies the PhotonVision pipeline index.
     */
    public void selectPipeline(int pipeline)
    {
        this.pipeline.setDouble(pipeline);
    }   //selectPipeline

    /**
     * This methods sets the LED mode.
     *
     * @param mode specifies LED mode.
     */
    public void setLEDMode(LEDMode mode)
    {
        ledMode.setDouble(mode.getValue());
    }   // setLEDMode

    /**
     * This method sets LED ON or OFF.
     *
     * @param enabled specifies true to turn ON LED, false to turn OFF.
     */
    public void setLEDEnabled(boolean enabled)
    {
        setLEDMode(enabled ? LEDMode.ON : LEDMode.OFF);
    }   //setLEDEnabled

    //
    // Implements FrcRemoteVisionProcessor abstract methods.
    //

    /**
     * This method gets the latest target data from vision processor.
     *
     * @return target data of the detected object, null if no object detected.
     */
    @Override
    protected TargetData<DetectedObject> getTargetData()
    {
        TargetData<DetectedObject> targetData = null;

        if (targetDetected())
        {
            targetData = new TargetData<>();
            targetData.timestamp = TrcTimer.getCurrentTime();
            targetData.data = getDetectedObject();
        }

        return targetData;
    }   //getTargetData

    /**
     * This method gets the target pose from the detected target data.
     *
     * @param targetData specifies the target data from which to get the target pose.
     * @return detected target pose.
     */
    @Override
    protected TrcPose3D getTargetPose(TargetData<?> targetData)
    {
        return ((DetectedObject) targetData.data).pose;
    }   //getTargetPose

}   //class FrcPhotonVisionRaw
