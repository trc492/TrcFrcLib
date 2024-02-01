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

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcOpenCvPipeline;
import TrcCommonLib.trclib.TrcPose3D;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionPerformanceMetrics;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;

/**
 * This class implements an AprilTag pipeline using OpenCV.
 */
public class FrcOpenCvAprilTagPipeline implements TrcOpenCvPipeline<TrcOpenCvDetector.DetectedObject<?>>
{
    private static final String moduleName = FrcOpenCvAprilTagPipeline.class.getSimpleName();

    /**
     * This class encapsulates info of the detected object. It extends TrcOpenCvDetector.DetectedObject that requires
     * it to provide a method to return the detected object rect and area.
     */
    public static class DetectedObject extends TrcOpenCvDetector.DetectedObject<AprilTagDetection>
    {
        /**
         * Constructor: Creates an instance of the object.
         *
         * @param aprilTagInfo specifies the detected april tag info.
         */
        public DetectedObject(String label, AprilTagDetection aprilTagInfo)
        {
            super(label, aprilTagInfo);
        }   //DetectedObject

        /**
         * This method calculates the rectangle of the detected AprilTag.
         *
         * @param at specifies the AprilTag info.
         * @return AprilTag rectangle.
         */
        public static Rect getDetectedRect(AprilTagDetection at)
        {
            double[] corners = at.getCorners();
            Point lowerLeftCorner = new Point(corners[0], corners[1]);
            Point lowerRightCorner = new Point(corners[2], corners[3]);
            Point upperRightCorner = new Point(corners[4], corners[5]);
            Point upperLeftCorner = new Point(corners[6], corners[7]);
            double width = ((upperRightCorner.x - upperLeftCorner.x) + (lowerRightCorner.x - lowerLeftCorner.x))/2.0;
            double height = ((lowerLeftCorner.y - upperLeftCorner.y) + (lowerRightCorner.y - upperRightCorner.y))/2.0;

            return new Rect((int)(at.getCenterX() - width/2.0), (int)(at.getCenterY() - height/2.0), (int) width, (int) height);
        }   //getDetectedRect

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getObjectRect()
        {
            return getDetectedRect(object);
        }   //getObjectRect

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        @Override
        public double getObjectArea()
        {
            return getDetectedRect(object).area();
        }   //getObjectArea

        /**
         * This method returns the pose of the detected object relative to the camera.
         *
         * @return pose of the detected object relative to camera.
         */
        @Override
        public TrcPose3D getObjectPose()
        {
            // Don't have enough info to generate this.
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

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @Override
        public String toString()
        {
            return "{id=" + object.getId() +
                   ",hamming=" + object.getHamming() +
                   ",decisionMargin=" + object.getDecisionMargin() +
                   ",center=" + object.getCenterX() + "/" + object.getCenterY() +
                   ",rect=" + getObjectRect();
        }   //toString

    }   //class DetectedObject

    private static final Scalar ANNOTATE_RED_COLOR = new Scalar(255,0,0,255);
    private static final Scalar ANNOTATE_GREEN_COLOR = new Scalar(0, 255, 0, 255);
    private static final Scalar ANNOTATE_BLUE_COLOR = new Scalar(7, 197, 235, 255);
    private static final Scalar ANNOTATE_WHITE_COLOR = new Scalar(255, 255, 255, 255);
    private static final int ANNOTATE_THICKNESS = 2;

    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final String tagFamily;
    private final AprilTagPoseEstimator.Config poseEstConfig;
    private final AprilTagDetector aprilTagDetector;
    // private final AprilTagPoseEstimator poseEstimator;
    private final Mat cameraMatrix;
    private final double tagSizeX, tagSizeY;
    private final Mat grayMat;
    private final Mat[] intermediateMats;

    private final AtomicReference<DetectedObject[]> detectedObjsUpdate = new AtomicReference<>();
    private int intermediateStep = 0;
    private boolean annotateEnabled = false;
    private TrcVisionPerformanceMetrics performanceMetrics = null;
    
    /**
     * Constructor: Create an instance of the object.
     *
     * @param tagFamily specifies the tag family.
     * @param detectorConfig specifies the detector configuration.
     * @param poseEstConfig specifies the pose estimator configuration (tagSize unit is in meter).
     */
    public FrcOpenCvAprilTagPipeline(
        String tagFamily, AprilTagDetector.Config detectorConfig, AprilTagPoseEstimator.Config poseEstConfig)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = moduleName + "." + tagFamily;
        this.tagFamily = tagFamily;
        this.poseEstConfig = poseEstConfig;
        // set up AprilTag detector
        aprilTagDetector = new AprilTagDetector();
        aprilTagDetector.addFamily(tagFamily);

        if (detectorConfig != null)
        {
            aprilTagDetector.setConfig(detectorConfig);
        }

        if (poseEstConfig != null)
        {
            // Set up Pose Estimator
            // poseEstimator = new AprilTagPoseEstimator(poseEstConfig);
            cameraMatrix = constructCameraMatrix();
            tagSizeX = tagSizeY = poseEstConfig.tagSize;
        }
        else
        {
            // poseEstimator = null;
            cameraMatrix = null;
            tagSizeX = tagSizeY = 0.0;
        }

        grayMat = new Mat();
        intermediateMats = new Mat[2];
        intermediateMats[0] = null;
        intermediateMats[1] = grayMat;
    }   //FrcOpenCvAprilTagPipeline

    /**
     * This method returns the tag family string.
     *
     * @return tag family string.
     */
    @Override
    public String toString()
    {
        return tagFamily;
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

    // /**
    //  * This method calculates the target position by using camera calibrated Homography.
    //  *
    //  * @param aprilTagObj specifies the detected AprilTag object.
    //  * @return target 3D position relative to the camera.
    //  */
    // public Pose3d getTargetPosition(DetectedObject aprilTagObj)
    // {
    //     Transform3d targetTransform = poseEstimator.estimate(aprilTagObj.object);
    //     Rotation3d targetRotation = targetTransform.getRotation();
    //     Pose3d targetPose = new Pose3d(
    //         targetTransform.getY() * TrcUtil.INCHES_PER_METER,
    //         targetTransform.getX() * TrcUtil.INCHES_PER_METER,
    //         targetTransform.getZ() * TrcUtil.INCHES_PER_METER,
    //         new Rotation3d(
    //             Math.toDegrees(targetRotation.getX()), Math.toDegrees(targetRotation.getY()),
    //             Math.toDegrees(targetRotation.getZ())));
    //     tracer.traceDebug(
    //         instanceName, "targetPose=%s, Yaw=%.1f", targetPose, targetPose.getRotation().getZ());
    //
    //     return targetPose;
    // }   //getTargetPosition

    //
    // Implements TrcOpenCvPipeline interface.
    //

    /**
     * This method is called to reset the state of the pipeline if any.
     */
    @Override
    public void reset()
    {
        if (performanceMetrics != null)
        {
            performanceMetrics.reset();
        }
        intermediateStep = 0;
    }   //reset

    /**
     * This method is called to process the input image through the pipeline.
     *
     * @param input specifies the input image to be processed.
     * @return array of detected objects.
     */
    @Override
    public DetectedObject[] process(Mat input)
    {
        DetectedObject[] detectedObjects;
        double startTime = TrcTimer.getCurrentTime();

        intermediateMats[0] = input;
        // Convert to grayscale.
        Imgproc.cvtColor(input, grayMat, Imgproc.COLOR_BGR2GRAY);
        AprilTagDetection[] detections = aprilTagDetector.detect(grayMat);
        if (performanceMetrics != null) performanceMetrics.logProcessingTime(startTime);

        detectedObjects = new DetectedObject[detections.length];
        for (int i = 0; i < detectedObjects.length; i++)
        {
            detectedObjects[i] = new DetectedObject(tagFamily + "." + detections[i].getId(), detections[i]);
            tracer.traceDebug(instanceName, "[" + i + "]" + " DetectedObj=" + detectedObjects[i]);
        }

        if (annotateEnabled)
        {
            Mat output = getIntermediateOutput(intermediateStep);
            // Scalar color = intermediateStep == 0? ANNOTATE_GREEN_COLOR: ANNOTATE_WHITE_COLOR;
            // annotateFrame(output, detectedObjects, color, ANNOTATE_THICKNESS);

            // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
            // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
            for (AprilTagDetection detection : detections)
            {
                SixDofPose pose = poseFromTrapezoid(detection.getCorners(), cameraMatrix, tagSizeX, tagSizeY);
                drawAxisMarker(output, tagSizeY/2.0, ANNOTATE_THICKNESS, pose.rvec, pose.tvec, cameraMatrix);
                draw3dCubeMarker(output, tagSizeX, tagSizeX, tagSizeY, ANNOTATE_THICKNESS, pose.rvec, pose.tvec, cameraMatrix);
                // Imgproc.rectangle(output, DetectedObject.getDetectedRect(detection), ANNOTATE_GREEN_COLOR, ANNOTATE_THICKNESS);
            }
        }
        detectedObjsUpdate.set(detectedObjects);

        return detectedObjects;
    }   //process

    /**
     * This method returns the array of detected objects.
     *
     * @return array of detected objects.
     */
    @Override
    public DetectedObject[] getDetectedObjects()
    {
        return detectedObjsUpdate.getAndSet(null);
    }   //getDetectedObjects

    /**
     * This method enables/disables image annotation of the detected object.
     *
     * @param enabled specifies true to enable annotation, false to disable.
     */
    @Override
    public void setAnnotateEnabled(boolean enabled)
    {
        annotateEnabled = enabled;
    }   //setAnnotateEnabled

    /**
     * This method checks if image annotation is enabled.
     *
     * @return true if annotation is enabled, false otherwise.
     */
    @Override
    public boolean isAnnotateEnabled()
    {
        return annotateEnabled;
    }   //isAnnotateEnabled

    /**
     * This method sets the intermediate mat of the pipeline as the video output mat.
     *
     * @param intermediateStep specifies the intermediate mat used as video output (1 is the original mat, 0 to
     *        disable video output if supported).
     */
    @Override
    public void setVideoOutput(int intermediateStep)
    {
        if (intermediateStep >= 0 && intermediateStep < intermediateMats.length)
        {
            this.intermediateStep = intermediateStep;
        }
    }   //setVideoOutput

    /**
     * This method cycles to the next intermediate mat of the pipeline as the video output mat.
     */
    @Override
    public void setNextVideoOutput()
    {
        intermediateStep = (intermediateStep + 1) % intermediateMats.length;
    }   //setNextVideoOutput

    /**
     * This method returns an intermediate processed frame. Typically, a pipeline processes a frame in a number of
     * steps. It may be useful to see an intermediate frame for a step in the pipeline for tuning or debugging
     * purposes.
     *
     * @param step specifies the intermediate step (step 1 is the original input frame).
     * @return processed frame of the specified step.
     */
    @Override
    public Mat getIntermediateOutput(int step)
    {
        Mat mat = null;

        if (step >= 0 && step <= intermediateMats.length)
        {
            mat = intermediateMats[step];
        }

        return mat;
    }   //getIntermediateOutput

    /**
     * This method returns the selected intermediate output Mat.
     *
     * @return selected output mat.
     */
    @Override
    public Mat getSelectedOutput()
    {
        return getIntermediateOutput(intermediateStep);
    }   //getSelectedOutput

    /**
     * This method constructs the camera matrix.
     */
    private Mat constructCameraMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //
        Mat camMatrix = new Mat(3, 3, CvType.CV_32FC1);

        camMatrix.put(0, 0, poseEstConfig.fx);
        camMatrix.put(0, 1, 0);
        camMatrix.put(0, 2, poseEstConfig.cx);

        camMatrix.put(1, 0, 0);
        camMatrix.put(1, 1, poseEstConfig.fy);
        camMatrix.put(1, 2, poseEstConfig.cy);

        camMatrix.put(2, 0, 0);
        camMatrix.put(2, 1, 0);
        camMatrix.put(2, 2, 1);

        return camMatrix;
    }   //constructMatrix

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param buf the RGB buffer on which to draw the marker
     * @param length the length of each of the marker 'poles'
     * @param thickness the thickness of the lines drawn.
     * @param rvec the rotation vector of the detection
     * @param tvec the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    private void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
            new Point3(0, 0, 0),
            new Point3(length,0,0),
            new Point3(0,length,0),
            new Point3(0,0,-length)
        );

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], ANNOTATE_RED_COLOR, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], ANNOTATE_GREEN_COLOR, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], ANNOTATE_BLUE_COLOR, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, ANNOTATE_WHITE_COLOR, -1);
    }   //drawAxisMarker

    /**
     * Draw a 3D cube marker on a detection.
     *
     * @param buf the RGB buffer on which to draw the marker
     * @param length the cube depth.
     * @param tagWidth the cube width.
     * @param tagHeight the cube height.
     * @param thickness the thickness of the lines drawn.
     * @param rvec the rotation vector of the detection
     * @param tvec the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    private void draw3dCubeMarker(
        Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
        //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
            new Point3(-tagWidth/2, tagHeight/2,0),
            new Point3( tagWidth/2, tagHeight/2,0),
            new Point3( tagWidth/2,-tagHeight/2,0),
            new Point3(-tagWidth/2,-tagHeight/2,0),
            new Point3(-tagWidth/2, tagHeight/2,-length),
            new Point3( tagWidth/2, tagHeight/2,-length),
            new Point3( tagWidth/2,-tagHeight/2,-length),
            new Point3(-tagWidth/2,-tagHeight/2,-length));

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Pillars
        for (int i = 0; i < 4; i++)
        {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], ANNOTATE_BLUE_COLOR, thickness);
        }

        // Base lines
        //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
        //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
        //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
        //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

        // Top lines
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], ANNOTATE_GREEN_COLOR, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], ANNOTATE_GREEN_COLOR, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], ANNOTATE_GREEN_COLOR, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], ANNOTATE_GREEN_COLOR, thickness);
    }   //draw3dCubeMarker

    /*
     * A simple container to hold both rotation and translation vectors, which together form a 6DOF pose.
     */
    static class SixDofPose
    {
        Mat rvec;
        Mat tvec;

        public SixDofPose()
        {
            rvec = new Mat();
            tvec = new Mat();
        }   //SixDofPose

        public SixDofPose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }   //class SixDofPose

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param pointsArray the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsizeX the original width of the tag
     * @param tagsizeY the original height of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    private SixDofPose poseFromTrapezoid(double[] pointsArray, Mat cameraMatrix, double tagsizeX , double tagsizeY)
    {
        Point[] points = {
            new Point(pointsArray[0], pointsArray[1]),
            new Point(pointsArray[2], pointsArray[3]),
            new Point(pointsArray[4], pointsArray[5]),
            new Point(pointsArray[6], pointsArray[7])};
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an 'ideal projection'
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Using this information, actually solve for pose
        SixDofPose pose = new SixDofPose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }   //poseFromTrapezoid

}  //class FrcOpenCvAprilTagPipeline
