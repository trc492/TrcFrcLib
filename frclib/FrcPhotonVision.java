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
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.List;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcPose3D;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcVisionPerformanceMetrics;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * This class implements vision detection using PhotonLib extending PhotonCamera.
 */
public abstract class FrcPhotonVision extends PhotonCamera
{
    private static final String moduleName = FrcPhotonVision.class.getSimpleName();
    private static final TrcDbgTrace staticTracer = new TrcDbgTrace();

    /**
     * This method is provided by the subclass to provide the target offset from ground so that vision can
     * accurately calculate the target position from the camera.
     *
     * @param target specifies the photon detected target.
     * @return target ground offset in inches.
     */
    public abstract double getTargetGroundOffset(PhotonTrackedTarget target);

    /**
     * This class encapsulates info of the detected object. It extends TrcOpenCvDetector.DetectedObject that requires
     * it to provide a method to return the detected object rect and area.
     */
    public static class DetectedObject implements TrcVisionTargetInfo.ObjectInfo
    {
        public final double timestamp;
        public final PhotonTrackedTarget target;
        public final Rect rect;
        public final double area;
        public final TrcPose3D targetPose;
        private final TrcPose3D pose3d;
        private final TrcPose2D pose2d;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param timestamp specifies the time stamp of the frame it was taken.
         * @param target specifies the photon detected target.
         * @param targetGroundOffset specifies the target ground offset in inches.
         * @param camGroundOffset specifies the camera ground offset in inches.
         * @param camPitch specifies the camera pitch from horizontal in degrees.
         */
        public DetectedObject(
            double timestamp, PhotonTrackedTarget target, double targetGroundOffset, double camGroundOffset,
            double camPitch)
        {
            this.timestamp = timestamp;
            this.target = target;
            this.rect = getRect(target);
            this.area = target.getArea();
            this.pose3d = getPose3d(target);
            this.pose2d = getPose2d(target, targetGroundOffset, camGroundOffset, camPitch);
            this.targetPose = getObjectPose();
        }   //DetectedObject

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @Override
        public String toString()
        {
            return "{time=" + timestamp +
                   ",pose3d=" + pose3d +
                   ",pose2d=" + pose2d +
                   ",rect=" + rect +
                   ",area=" + area +
                   ",target=" + target + "}";
        }   //toString

        /**
         * This method returns the rect of the detected object.
         *
         * @param target specifies the detected target.
         * @return rect of the detected target.
         */
        public static Rect getRect(PhotonTrackedTarget target)
        {
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
                staticTracer.traceDebug(
                    moduleName + ".Id" + target.getFiducialId(),
                    " UpperLeft: x=" + upperLeftCorner.x + ", y=" + upperLeftCorner.y +
                    "\nUpperRight: x=" + upperRightCorner.x + ", y=" + upperRightCorner.y +
                    "\n LowerLeft: x=" +  lowerLeftCorner.x + ", y=" + lowerLeftCorner.y +
                    "\nLowerRight: x=" +  lowerRightCorner.x + ", y=" + lowerRightCorner.y);
            }

            return rect;
        }   //getRect

        /**
         * This method calculates the target pose of the detected object.
         *
         * @param target specifies the detected target.
         * @return target pose of the detected target.
         */
        public static TrcPose3D getPose3d(PhotonTrackedTarget target)
        {
            TrcPose3D pose;
            Transform3d targetTransform3d = target.getBestCameraToTarget();
            Translation3d targetTranslation = targetTransform3d.getTranslation();
            Rotation3d targetRotation = targetTransform3d.getRotation();

            pose = new TrcPose3D(
                -targetTranslation.getY()*TrcUtil.INCHES_PER_METER,
                targetTranslation.getX()*TrcUtil.INCHES_PER_METER,
                targetTranslation.getZ()*TrcUtil.INCHES_PER_METER,
                -Math.toDegrees(targetRotation.getZ()),
                Math.toDegrees(targetRotation.getY()),
                Math.toDegrees(targetRotation.getX()));

            return pose;
        }   //getPose3d

        /**
         * This method calculates the pose of the detected object given the yaw, pitch and height offset of the object
         * from ground.
         *
         * @param target specifies the detected target.
         * @param targetGroundOffset specifies the target ground offset in inches.
         * @param camGroundOffset specifies the camera ground offset in inches.
         * @param camPitch specifies the camera pitch from horizontal in degrees.
         * @return a 2D pose of the detected object from the camera.
         */
        public static TrcPose2D getPose2d(
            PhotonTrackedTarget target, double targetGroundOffset, double camGroundOffset, double camPitch)
        {
            double targetYawDegrees = target.getYaw();
            double targetPitchDegrees = target.getPitch();
            double targetYawRadians = Math.toRadians(targetYawDegrees);
            double targetPitchRadians = Math.toRadians(targetPitchDegrees);
            double camPitchRadians = Math.toRadians(camPitch);
            double targetDistanceInches =
                (targetGroundOffset - camGroundOffset)/Math.tan(camPitchRadians + targetPitchRadians);

            return new TrcPose2D(
                targetDistanceInches * Math.sin(targetYawRadians),
                targetDistanceInches * Math.cos(targetYawRadians),
                targetYawDegrees);
        }   //getPose2d

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getObjectRect()
        {
            return rect.clone();
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
            TrcPose3D pose;

            if (pose3d.x == 0.0 && pose3d.y == 0.0 && pose3d.z == 0.0 &&
                pose3d.yaw == 0.0 && pose3d.pitch == 0.0 && pose3d.roll == 0.0)
            {
                pose = new TrcPose3D(pose2d.x, pose2d.y, 0.0, pose2d.angle, 0.0, 0.0);
            }
            else
            {
                pose = pose3d.clone();
            }

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

    protected final TrcDbgTrace tracer;
    protected final String instanceName;
    private final double camGroundOffset;
    private final double camPitch;
    private TrcVisionPerformanceMetrics performanceMetrics = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraName specifies the network table name that PhotonVision is broadcasting information over.
     * @param camGroundOffset specifies the camera ground offset in inches.
     * @param camPitch specifies the camera pitch from horizontal in degrees.
     */
    public FrcPhotonVision(String cameraName, double camGroundOffset, double camPitch)
    {
        super(cameraName);
        this.tracer = new TrcDbgTrace();
        this.instanceName = cameraName;
        this.camGroundOffset = camGroundOffset;
        this.camPitch = camPitch;
    }   //FrcPhotonVision

    /**
     * This method returns the photon camera name.
     *
     * @return photon camera name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method enables/disables performance metrics.
     *
     * @param enabled specifies true to enable performance metrics, false to disable.
     */
    public void setPerformanceMetricsEnabled(boolean enabled)
    {
        if (performanceMetrics == null && enabled)
        {
            performanceMetrics = new TrcVisionPerformanceMetrics(instanceName);
        }
        else if (performanceMetrics != null && !enabled)
        {
            performanceMetrics = null;
        }
    }   //setPerformanceMetricsEnabled

    /**
     * This method prints the performance metrics to the trace log.
     */
    public void printPerformanceMetrics()
    {
        if (performanceMetrics != null)
        {
            performanceMetrics.printMetrics(tracer);
        }
    }   //printPerformanceMetrics

    /**
     * This method returns the array of detected objects.
     *
     * @return array of detected objects.
     */
    public DetectedObject[] getDetectedObjects()
    {
        DetectedObject[] detectedObjs = null;
        double startTime = TrcTimer.getCurrentTime();
        PhotonPipelineResult result = getLatestResult();
        if (performanceMetrics != null) performanceMetrics.logProcessingTime(startTime);

        if (result.hasTargets())
        {
            List<PhotonTrackedTarget> targets = result.getTargets();
            double timestamp = result.getTimestampSeconds();

            detectedObjs = new DetectedObject[targets.size()];
            for (int i = 0; i < targets.size(); i++)
            {
                PhotonTrackedTarget target = targets.get(i);
                detectedObjs[i] = new DetectedObject(
                    timestamp, target, getTargetGroundOffset(target), camGroundOffset, camPitch);
                tracer.traceDebug(instanceName, "[" + i + "] DetectedObj=" + detectedObjs[i]);
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
        DetectedObject bestDetectedObj = null;
        double startTime = TrcTimer.getCurrentTime();
        PhotonPipelineResult result = getLatestResult();
        if (performanceMetrics != null) performanceMetrics.logProcessingTime(startTime);

        if (result.hasTargets())
        {
            PhotonTrackedTarget target = result.getBestTarget();
            bestDetectedObj = new DetectedObject(
                result.getTimestampSeconds(), target, getTargetGroundOffset(target), camGroundOffset, camPitch);
            tracer.traceInfo(instanceName, "DetectedObj=" + bestDetectedObj);
        }

        return bestDetectedObj;
    }   //getBestDetectedObject

    /**
     * This method uses the PhotonVision Pose Estimator to get an estimated absolute field position of the robot.
     *
     * @return absolute robot field position, can be null if not provided.
     */
    public TrcPose2D getRobotFieldPosition(Transform3d cameraToRobot)
    {
        TrcPose2D robotPose = null;
        double startTime = TrcTimer.getCurrentTime();
        PhotonPipelineResult result = getLatestResult();
        if (performanceMetrics != null) performanceMetrics.logProcessingTime(startTime);

        PNPResult estimatedPose = result.getMultiTagResult().estimatedPose;
        if (estimatedPose.isPresent)
        {
            Transform3d fieldToRobot = estimatedPose.best.plus(cameraToRobot);
            Translation3d translation = fieldToRobot.getTranslation();
            Rotation3d rotation = fieldToRobot.getRotation();
            robotPose = new TrcPose2D(
                -translation.getY()*TrcUtil.INCHES_PER_METER,
                translation.getX()*TrcUtil.INCHES_PER_METER,
                -Math.toDegrees(rotation.getZ()));
        }

        return robotPose;
    }   //getRobotFieldPosition

}   //class FrcPhotonVision
