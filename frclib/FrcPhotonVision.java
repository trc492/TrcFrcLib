/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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

import org.opencv.core.Rect;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicReference;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcVisionPerformanceMetrics;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcCommonLib.trclib.TrcTaskMgr.TaskType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * This class implements vision detection using PhotonLib extending PhotonCamera.
 */
public class FrcPhotonVision extends PhotonCamera
{
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private static final boolean debugEnabled = false;

    /**
     * This class encapsulates info of the detected object. It extends TrcOpenCvDetector.DetectedObject that requires
     * it to provide a method to return the detected object rect and area.
     */
    public static class DetectedObject implements TrcVisionTargetInfo.ObjectInfo
    {
        public final double timestamp;
        public final PhotonTrackedTarget target;
        public final Transform3d targetTransform;
        public final Pose3d targetPose3D;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param timestamp specifies the time stamp of the frame it was taken.
         * @param target specifies the photon detected target.
         */
        public DetectedObject(double timestamp, PhotonTrackedTarget target)
        {
            this.timestamp = timestamp;
            this.target = target;
            targetTransform = target.getBestCameraToTarget();
            targetPose3D = new Pose3d(
                -targetTransform.getY() * TrcUtil.INCHES_PER_METER, targetTransform.getX() * TrcUtil.INCHES_PER_METER,
                targetTransform.getZ() * TrcUtil.INCHES_PER_METER,
                new Rotation3d(0.0, target.getPitch(), target.getYaw()));
        }   //DetectedObject

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "{timestamp=%.3f,pose=(x=%.1f,y=%.1f,z=%.1f,pitch=%.1f,yaw=%.1f,skew=%.1f),rect=%s,area=%.1f}",
                timestamp, targetPose3D.getX(), targetPose3D.getY(), targetPose3D.getZ(),
                target.getPitch(), target.getYaw(), target.getSkew(), getRect(), target.getArea());
        }   //toString

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getRect()
        {
            final String funcName = "getRect";
            Rect rect = null;
            List<TargetCorner> corners = target.getDetectedCorners();
            TargetCorner lowerLeftCorner = null;
            TargetCorner lowerRightCorner = null;
            TargetCorner upperLeftCorner = null;
            TargetCorner upperRightCorner = null;

            if (corners != null && corners.size() >= 4)
            {
                lowerLeftCorner = corners.get(0);
                lowerRightCorner = corners.get(1);
                upperRightCorner = corners.get(2);
                upperLeftCorner = corners.get(3);
            }
            else if ((corners = target.getMinAreaRectCorners()) != null && corners.size() >= 4)
            {
                upperLeftCorner = corners.get(0);
                upperRightCorner = corners.get(1);
                lowerRightCorner = corners.get(2);
                lowerLeftCorner = corners.get(3);
            }

            if (upperLeftCorner != null)
            {
                double width =
                    ((upperRightCorner.x - upperLeftCorner.x) + (lowerRightCorner.x - lowerLeftCorner.x))/2.0;
                double height =
                    ((lowerLeftCorner.y - upperLeftCorner.y) + (lowerRightCorner.y - upperRightCorner.y))/2.0;
                rect = new Rect((int)upperLeftCorner.x, (int)upperLeftCorner.y, (int)width, (int)height);
                if (debugEnabled)
                {
                    globalTracer.traceInfo(
                        funcName, " UpperLeft: x=%.1f, y=%.1f", upperLeftCorner.x, upperLeftCorner.y);
                    globalTracer.traceInfo(
                        funcName, "UpperRight: x=%.1f, y=%.1f", upperRightCorner.x, upperRightCorner.y);
                    globalTracer.traceInfo(
                        funcName, " LowerLeft: x=%.1f, y=%.1f", lowerLeftCorner.x, lowerLeftCorner.y);
                    globalTracer.traceInfo(
                        funcName, "LowerRight: x=%.1f, y=%.1f", lowerRightCorner.x, lowerRightCorner.y);
                }
            }

            return rect;
        }   //getRect

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        @Override
        public double getArea()
        {
            return target.getArea();
        }   //getArea

    }   //class DetectedObject

    private final TrcVisionPerformanceMetrics performanceMetrics = new TrcVisionPerformanceMetrics();
    private final double camHeightInches;
    private final double camPitchRadians;
    private final TrcDbgTrace tracer;
    private final TrcTaskMgr.TaskObject visionTaskObj;
    private AtomicReference<DetectedObject[]> lastDetectedObjects = new AtomicReference<>();
    private AtomicReference<DetectedObject> lastDetectedBestObject = new AtomicReference<>();
    private TrcEvent completionEvent = null;
    private double expireTime = 0.0;
    private boolean detectBest = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraName specifies the photon vision camera name.
     * @param camHeight specifies the camera height from the ground in inches.
     * @param camPitch specifies the camera pitch angle from horizontal in degrees.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public FrcPhotonVision(String cameraName, double camHeight, double camPitch, TrcDbgTrace tracer)
    {
        super(cameraName);
        this.camHeightInches = camHeight;
        this.camPitchRadians = Math.toRadians(camPitch);
        this.tracer = tracer;
        visionTaskObj = TrcTaskMgr.createTask(cameraName + ".visionTask", this::visionTask);
    }   //FrcPhotonVision

    /**
     * This method returns the photon camera name.
     *
     * @return photon camera name.
     */
    @Override
    public String toString()
    {
        return getName();
    }   //toString

    /**
     * This method cancels pending detection request if any.
     */
    public void cancel()
    {
        visionTaskObj.unregisterTask();
    }   //cancel

    /**
     * This method start a detection request. If there is already a pending request, this will fail and return false.
     *
     * @param event specifies the event to signal if detection request is completed.
     * @param timeout specifies the maximum timeout allowed for the detection. If failed to detect objects past
     *        timeout, event will be signaled. Caller is responsible to check if there is valid detection result.
     * @retrun true if detection request is started, false if there is already a pending request.
     */
    public boolean detectObjects(TrcEvent event, double timeout)
    {
        boolean success = false;

        if (event == null || timeout <= 0.0)
        {
            throw new IllegalArgumentException("Must provide a non-null event and a valid timeout.");
        }

        if (!visionTaskObj.isRegistered())
        {
            lastDetectedObjects.set(null);
            completionEvent = event;
            expireTime = TrcTimer.getCurrentTime() + timeout;
            detectBest = false;
            visionTaskObj.registerTask(TaskType.PRE_PERIODIC_TASK);
            success = true;
        }

        return success;
    }   //detectObjects

    /**
     * This method start a detection request. If there is already a pending request, this will fail and return false.
     *
     * @param event specifies the event to signal if detection request is completed.
     * @param timeout specifies the maximum timeout allowed for the detection. If failed to detect objects past
     *        timeout, event will be signaled. Caller is responsible to check if there is valid detection result.
     * @retrun true if detection request is started, false if there is already a pending request.
     */
    public boolean detectBestObject(TrcEvent event, double timeout)
    {
        boolean success = false;

        if (event == null || timeout <= 0.0)
        {
            throw new IllegalArgumentException("Must provide a non-null event and a valid timeout.");
        }

        if (!visionTaskObj.isRegistered())
        {
            lastDetectedBestObject.set(null);
            completionEvent = event;
            expireTime = TrcTimer.getCurrentTime() + timeout;
            detectBest = true;
            visionTaskObj.registerTask(TaskType.PRE_PERIODIC_TASK);
            success = true;
        }

        return success;
    }   //detectBestObject

    /**
     * This method returns the last vision detected objects. This call consumes the detected objects.
     *
     * @return array of last detected objects.
     */
    public DetectedObject[] getLastDetectedObjects()
    {
        return lastDetectedObjects.getAndSet(null);
    }   //getLastDetectedObjects

    /**
     * This method returns the last vision detected best object. This call consumes the detected object.
     *
     * @return last detected best object.
     */
    public DetectedObject getLastDetectedBestObject()
    {
        return lastDetectedBestObject.getAndSet(null);
    }   //getLastDetectedBestObject

    /**
     * This method returns the array of detected objects.
     *
     * @return array of detected objects.
     */
    public DetectedObject[] getDetectedObjects()
    {
        final String funcName = "getDetectedObjects";
        DetectedObject[] detectedObjs = null;
        double startTime = TrcTimer.getCurrentTime();
        PhotonPipelineResult result = getLatestResult();
        performanceMetrics.logProcessingTime(startTime);
        performanceMetrics.printMetrics(tracer);

        if (result.hasTargets())
        {
            List<PhotonTrackedTarget> targets = result.getTargets();
            double timestamp = result.getTimestampSeconds();
            detectedObjs = new DetectedObject[targets.size()];

            for (int i = 0; i < targets.size(); i++)
            {
                detectedObjs[i] = new DetectedObject(timestamp, targets.get(i));
                if (tracer != null)
                {
                    tracer.traceInfo(
                        funcName, "[%d: %.3f] DetectedObj=%s", i, TrcTimer.getModeElapsedTime(), detectedObjs[i]);
                }
            }
        }

        return detectedObjs;
    }   //getDetectedObjects

    /**
     * This method returns the best detected object.
     *
     * @return best detected object.
     */
    public DetectedObject getBestDetectedObject()
    {
        final String funcName = "getBestDetectedObject";
        DetectedObject bestDetectedObj = null;
        PhotonPipelineResult result = getLatestResult();

        if (result.hasTargets())
        {
            PhotonTrackedTarget target = result.getBestTarget();
            bestDetectedObj = new DetectedObject(result.getTimestampSeconds(), target);
            if (tracer != null)
            {
                tracer.traceInfo(
                    funcName, "[%.3f] DetectedObj=%s", TrcTimer.getModeElapsedTime(), bestDetectedObj);
            }
        }

        return bestDetectedObj;
    }   //getBestDetectedObject

    /**
     * This method calculates the pose of the detected object given the height offset of the object from ground.
     *
     * @param detectedObj specifies the detected object.
     * @param targetHeightInches specifies the height offset of the object from ground.
     * @return a 2D pose of the detected object from the camera.
     */
    public TrcPose2D getTargetPose2D(DetectedObject detectedObj, double targetHeightInches)
    {
        final String funcName = "getTargetPose2D";
        double targetYawDegrees = detectedObj.target.getYaw();
        double targetYawRadians = Math.toRadians(targetYawDegrees);
        double targetPitchRadians = Math.toRadians(detectedObj.target.getPitch());
        double targetDistanceInches =
            (targetHeightInches - camHeightInches)/Math.tan(camPitchRadians + targetPitchRadians);
        TrcPose2D targetPose = new TrcPose2D(
            targetDistanceInches * Math.sin(targetYawRadians), targetDistanceInches * Math.cos(targetYawRadians),
            targetYawDegrees);

        if (tracer != null)
        {
            tracer.traceInfo(
                funcName, "[%.3f] TargetPose=%s", TrcTimer.getModeElapsedTime(), targetPose);
        }

        return targetPose;
    }   //getTargetPose2D

    /**
     * This methods is called periodically to check if vision has detected objects.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void visionTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        boolean detected = false;

        if (detectBest)
        {
            DetectedObject detectedBestObj = getBestDetectedObject();
            detected = lastDetectedBestObject != null;
            lastDetectedBestObject.set(detectedBestObj);
        }
        else
        {
            DetectedObject[] detectedObjs = getDetectedObjects();
            detected = detectedObjs != null;
            lastDetectedObjects.set(detectedObjs);
        }

        if (detected || TrcTimer.getCurrentTime() >= expireTime)
        {
            // Either we have detected objects or we have timed out, signal completion and disable task. 
            completionEvent.signal();
            completionEvent = null;
            expireTime = 0.0;
            visionTaskObj.unregisterTask();
        }
    }   //visionTask

}   //class FrcPhotonVision
