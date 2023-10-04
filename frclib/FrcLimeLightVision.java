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

import edu.wpi.first.networktables.NetworkTableEntry;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcPose3D;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionTargetInfo;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

import org.opencv.core.Rect;

public class FrcLimeLightVision extends FrcRemoteVisionProcessor
{
    /**
     * This class encapsulates info of the detected object. It extends TrcVisionTargetInfo.ObjectInfo that requires
     * it to provide a method to return the detected object rect.
     */
    public static class DetectedObject implements TrcVisionTargetInfo.ObjectInfo
    {
        public RelativePose relativePose;
        public double area;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param object specifies the contour of the object.
         */
        public DetectedObject(RelativePose relPose, double area)
        {
            this.relativePose = relPose;
            this.area = area;
        }   //DetectedObject

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @Override
        public String toString()
        {
            return "{relPose=" + relativePose + ",area=" + area + "}";
        }   //toString

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getRect()
        {
            throw new UnsupportedOperationException("LimeLight vision does not provide object rectangle.");
        }   //getRect

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        @Override
        public double getArea()
        {
            return area;
        }   //getArea

        /**
         * This method returns the pose of the detected object relative to the camera.
         *
         * @return pose of the detected object relative to camera.
         */
        @Override
        public TrcPose3D getObjectPose()
        {
            return null;
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

    public enum RingLightMode
    {
        AUTO(0), OFF(1), BLINK(2), ON(3);

        private int value;

        RingLightMode(int value)
        {
            this.value = value;
        }

        public int getValue()
        {
            return value;
        }
    }

    private final TrcDbgTrace tracer;
    private NetworkTableEntry tv, heading;
    private NetworkTableEntry ledMode, pipeline;
    private DoubleSupplier depthSupplier = () -> 0.0;

    public FrcLimeLightVision(String instanceName, String depthInput, DoubleUnaryOperator depthApproximator, TrcDbgTrace tracer)
    {
        this(instanceName, tracer);
        setDepthApproximator(depthInput, depthApproximator);
    }

    public FrcLimeLightVision(String instanceName, TrcDbgTrace tracer)
    {
        super(instanceName, instanceName);
        this.tracer = tracer;
        tv = networkTable.getEntry("tv");
        heading = networkTable.getEntry("tx");
        ledMode = networkTable.getEntry("ledMode");
        pipeline = networkTable.getEntry("pipeline");
    }   //FrcLimeLightVision

    /**
     * This method returns the detected object.
     *
     * @return detected object.
     */
    public DetectedObject getDetectedObject()
    {
        final String funcName = "getDetectedObject";
        DetectedObject detectedObj = null;

        RelativePose pose = getLastPose();
        if (pose != null)
        {
            detectedObj = new DetectedObject(pose, getTargetArea());
            if (tracer != null)
            {
                tracer.traceInfo(funcName, "DetectedObj=%s", detectedObj);
            }
        }

        return detectedObj;
    }   //getDetectedObject

    /**
     * This method returns the selected LimeLight pipeline.
     */
    public int getSelectedPipeline()
    {
        return (int) pipeline.getDouble(0.0);
    }   //getSelectedPipeline

    public void selectPipeline(int pipeline)
    {
        this.pipeline.setDouble(pipeline);
    }   //selectPipeline


    public double getTargetDepth()
    {
        return targetDetected() ? depthSupplier.getAsDouble() : 0.0;
    }

    public void setDepthApproximator(String input, DoubleUnaryOperator depthApproximator)
    {
        depthSupplier = () -> depthApproximator.applyAsDouble(networkTable.getEntry(input).getDouble(0.0));
    }

    public boolean targetDetected()
    {
        return tv.getDouble(0.0) == 1.0;
    }

    public double getHeading()
    {
        return targetDetected() ? heading.getDouble(0.0) : 0.0;
    }

    public double getElevation()
    {
        return targetDetected() ? get("ty") : 0.0;
    }

    public double getTargetArea()
    {
        return get("ta");
    }

    public double getTargetHeight()
    {
        return get("tvert");
    }

    @Override
    public void setRingLightEnabled(boolean enabled)
    {
        setRingLightEnabled(enabled ? RingLightMode.ON : RingLightMode.OFF);
    }

    public void setRingLightEnabled(RingLightMode mode)
    {
        ledMode.setDouble(mode.getValue());
    }

    @Override
    protected RelativePose processData()
    {
        if (!targetDetected())
        {
            return null;
        }

        RelativePose pose = new RelativePose();
        pose.time = TrcTimer.getCurrentTime();
        pose.theta = getHeading();
        pose.r = depthSupplier.getAsDouble();
        pose.x = Math.sin(Math.toRadians(pose.theta)) * pose.r;
        pose.y = Math.cos(Math.toRadians(pose.theta)) * pose.r;
        pose.objectYaw = 0.0;
        return pose;
    }

}   //class FrcLimeLightVision
