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
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionPerformanceMetrics;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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
    public class DetectedObject implements TrcVisionTargetInfo.ObjectInfo
    {
        public final double timestamp;
        public final PhotonTrackedTarget target;
        public final Rect rect;
        public final double area;
        public final TrcPose2D targetPose;
        public final TrcPose2D robotPose;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param timestamp specifies the time stamp of the frame it was taken.
         * @param target specifies the photon detected target.
         * @param robotToCamera specifies the Transform3d of the camera position on the robot.
         * @param robotPose specifies the estimated robot pose.
         */
        public DetectedObject(
            double timestamp, PhotonTrackedTarget target, Transform3d robotToCamera, TrcPose2D robotPose)
        {
            this.timestamp = timestamp;
            this.target = target;
            this.rect = getRect(target);
            this.area = target.getArea();
            this.targetPose = getTargetPose(target, robotToCamera);
            this.robotPose = robotPose;
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
                   ",pose=" + targetPose +
                   ",rect=" + rect +
                   ",area=" + area +
                   ",target=" + target +
                   ",robotPose=" + robotPose + "}";
        }   //toString

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
        public TrcPose2D getObjectPose()
        {
            return targetPose.clone();
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

        /**
         * This method returns the rect of the detected object.
         *
         * @param target specifies the detected target.
         * @return rect of the detected target.
         */
        private Rect getRect(PhotonTrackedTarget target)
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
         * This method calculates the target pose of the detected object. If PhotonVision 3D model is enabled
         * (transform3d.translation3d is not zero), it will use the 3D info to calculate the detected object pose
         * projected on the ground. Otherwise, it will use the 2D model (yaw and pitch angles).
         *
         * @param target specifies the detected target.
         * @param robotToCamera specifies the Transform3d of the camera position on the robot.
         * @return target pose of the detected target.
         */
        private TrcPose2D getTargetPose(PhotonTrackedTarget target, Transform3d robotToCamera)
        {
            TrcPose2D targetPose = null;
            Translation3d camToTargetTranslation = target.getBestCameraToTarget().getTranslation();

            if (camToTargetTranslation.getX() != 0.0 || camToTargetTranslation.getY() != 0.0 ||
                camToTargetTranslation.getZ() != 0.0)
            {
                // Use PhotonVision 3D model
                Transform3d robotToTarget = robotToCamera.plus(target.getBestCameraToTarget());
                Translation2d targetTranslation = robotToTarget.getTranslation().toTranslation2d();
                Rotation2d targetRotation = robotToTarget.getRotation().toRotation2d();
                targetPose = new TrcPose2D(
                    Units.metersToInches(-targetTranslation.getY()),
                    Units.metersToInches(targetTranslation.getX()),
                    -targetRotation.getDegrees());
            }
            else
            {
                // Use PhotonVision 2D model.
                double targetYawDegrees = target.getYaw();
                double targetPitchDegrees = target.getPitch();
                double targetYawRadians = Math.toRadians(targetYawDegrees);
                double targetPitchRadians = Math.toRadians(targetPitchDegrees);
                double camPitchRadians = -robotToCamera.getRotation().getY();
                double targetDistanceInches =
                    (getTargetGroundOffset(target) - Units.metersToInches(robotToCamera.getTranslation().getZ())) /
                    Math.tan(camPitchRadians + targetPitchRadians);
                targetPose = new TrcPose2D(
                    targetDistanceInches * Math.sin(targetYawRadians),
                    targetDistanceInches * Math.cos(targetYawRadians),
                    targetYawDegrees);
            }

            return targetPose;
        }   //getTargetPose

    }   //class DetectedObject

    protected final TrcDbgTrace tracer;
    protected final String instanceName;
    private final Transform3d robotToCamera;
    private TrcVisionPerformanceMetrics performanceMetrics = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraName specifies the network table name that PhotonVision is broadcasting information over.
     * @param robotToCamera specifies the Transform3d of the camera position on the robot.
     */
    public FrcPhotonVision(String cameraName, Transform3d robotToCamera)
    {
        super(cameraName);
        this.tracer = new TrcDbgTrace();
        this.instanceName = cameraName;
        this.robotToCamera = robotToCamera;
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
                    timestamp, target, robotToCamera, getRobotEstimatedPose(result, robotToCamera));
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
                result.getTimestampSeconds(), target, robotToCamera, getRobotEstimatedPose(result, robotToCamera));
            tracer.traceDebug(instanceName, "DetectedObj=" + bestDetectedObj);
        }

        return bestDetectedObj;
    }   //getBestDetectedObject

    /**
     * This method returns the detected AprilTag object.
     *
     * @param aprilTagId specifies the AprilTag ID to look for, -1 if looking for any AprilTag.
     * @return detected AprilTag object.
     */
    public DetectedObject getDetectedAprilTag(int aprilTagId)
    {
        DetectedObject detectedAprilTag = null;
        double startTime = TrcTimer.getCurrentTime();
        PhotonPipelineResult result = getLatestResult();
        if (performanceMetrics != null) performanceMetrics.logProcessingTime(startTime);

        if (result.hasTargets())
        {
            List<PhotonTrackedTarget> targets = result.getTargets();
            double timestamp = result.getTimestampSeconds();

            for (PhotonTrackedTarget target: targets)
            {
                // Return the detected AprilTag with matching ID or the first one if no ID is provided.
                if (aprilTagId == -1 || aprilTagId == target.getFiducialId())
                {
                    detectedAprilTag = new DetectedObject(
                        timestamp, target, robotToCamera, getRobotEstimatedPose(result, robotToCamera));
                    tracer.traceDebug(instanceName, "DetectedAprilTag=" + detectedAprilTag);
                    break;
                }
            }
        }

        return detectedAprilTag;
    }   //getDetectedAprilTag

    /**
     * This method uses the PhotonVision Pose Estimator to get an estimated absolute field position of the robot.
     *
     * @param result specifies the latest pipeline result.
     * @param robotToCamera specifies the Transform3d position of the camera from the robot center.
     * @return absolute robot field position, can be null if not provided.
     */
    public TrcPose2D getRobotEstimatedPose(PhotonPipelineResult result, Transform3d robotToCamera)
    {
        TrcPose2D robotPose = null;
        PNPResult estimatedPose = result.getMultiTagResult().estimatedPose;

        if (estimatedPose.isPresent)
        {
            Transform3d fieldToRobot = estimatedPose.best.plus(robotToCamera.inverse());
            Translation2d translation = fieldToRobot.getTranslation().toTranslation2d();
            Rotation2d rotation = fieldToRobot.getRotation().toRotation2d();

            robotPose = new TrcPose2D(
                Units.metersToInches(-translation.getY()),
                Units.metersToInches(translation.getX()),
                rotation.getDegrees());
        }

        return robotPose;
    }   //getRobotEstimatedPose

    /**
     * This method uses the PhotonVision Pose Estimator to get an estimated absolute field position of the robot.
     *
     * @param robotToCamera specifies the Transform3d position of the camera from the robot center.
     * @return absolute robot field position, can be null if not provided.
     */
    public TrcPose2D getRobotEstimatedPose(Transform3d robotToCamera)
    {
        double startTime = TrcTimer.getCurrentTime();
        PhotonPipelineResult result = getLatestResult();
        if (performanceMetrics != null) performanceMetrics.logProcessingTime(startTime);

        return getRobotEstimatedPose(result, robotToCamera);
    }   //getRobotEstimatedPose

    /**
     * This method calculates the robot's field position by subtracting the AprilTag's field position by the AprilTag
     * position from the camera and the camera position on the robot.
     *
     * @param aprilTagFieldPose specifies the AprilTag's field position.
     * @param aprilTagTargetPose specifies the AprilTag's position from the camera.
     * @param robotToCamera specifies the camera's position on the robot.
     * @return robot's field position.
     */
    public TrcPose2D getRobotPoseFromAprilTagFieldPose(
        TrcPose2D aprilTagFieldPose, TrcPose2D aprilTagTargetPose, TrcPose2D robotToCamera)
    {
        return aprilTagFieldPose.subtractRelativePose(aprilTagTargetPose).subtractRelativePose(robotToCamera);
    }   //getRobotPoseFromAprilTagFieldPose

    /**
     * This method calculates the target pose with an offset from the given AprilTag pose.
     *
     * @param aprilTagFieldPose3d specifies the AprilTag 3D field pose.
     * @param xOffset specifies the x-offset from AprilTag in inches.
     * @param yOffset specifies the y-offset from AprilTag in inches.
     * @param targetAngle specifies the target field angle in degrees.
     * @return calculated target pose.
     */
    public TrcPose2D getTargetPoseOffsetFromAprilTag(Pose3d aprilTagFieldPose3d, double xOffset, double yOffset, double targetAngle)
    {
        Transform2d offset = new Transform2d(
            new Translation2d(Units.inchesToMeters(yOffset), Units.inchesToMeters(-xOffset)),
            new Rotation2d(Units.degreesToRadians(targetAngle)));
        Pose2d targetPose2d = aprilTagFieldPose3d.toPose2d().plus(offset);
        return new TrcPose2D(targetPose2d.getX(), -targetPose2d.getY(), -targetPose2d.getRotation().getDegrees());
    }   //getTargetPoseOffsetFromAprilTag

}   //class FrcPhotonVision
