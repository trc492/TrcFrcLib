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

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Locale;
import java.util.concurrent.atomic.AtomicReference;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcOpenCvPipeline;
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
        public DetectedObject(AprilTagDetection aprilTagInfo)
        {
            super(aprilTagInfo);
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
        public Rect getRect()
        {
            return getDetectedRect(object);
        }   //getRect

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        @Override
        public double getArea()
        {
            return getDetectedRect(object).area();
        }   //getArea

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US, "{id=%d,hamming=%d,decisionMargin=%.1f,center=%.1f/%.1f,rect=%s}",
                object.getId(), object.getHamming(), object.getDecisionMargin(), object.getCenterX(), object.getCenterY(), getRect());
        }   //toString

    }   //class DetectedObject

    private static final Scalar ANNOTATE_RECT_COLOR = new Scalar(0, 255, 0, 255);
    private static final Scalar ANNOTATE_RECT_WHITE = new Scalar(255, 255, 255, 255);
    private static final int ANNOTATE_RECT_THICKNESS = 3;
    // private static final float DEF_DECIMATION = 3.0f;
    // private static final int NUM_THREADS = 3;
    // private static final Scalar RED = new Scalar(255,0,0,255);
    // private static final Scalar BLUE = new Scalar(7, 197, 235, 255);
    // private static final Scalar WHITE = new Scalar(255,255,255,255);

    private final String tagFamily;
    private final TrcDbgTrace tracer;
    private final AprilTagDetector aprilTagDetector;
    private final AprilTagPoseEstimator poseEstimator;
    private final Mat grayMat;
    private final Mat[] intermediateMats;

    private final TrcVisionPerformanceMetrics performanceMetrics = new TrcVisionPerformanceMetrics();
    private final AtomicReference<DetectedObject[]> detectedObjsUpdate = new AtomicReference<>();
    private int intermediateStep = 0;
    private boolean annotate = false;
    
    /**
     * Constructor: Create an instance of the object.
     *
     * @param tagFamily specifies the tag family.
     * @param detectorConfig specifies the detector configuration.
     * @param poseEstConfig specifies the pose estimator configuration.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public FrcOpenCvAprilTagPipeline(
        String tagFamily, AprilTagDetector.Config detectorConfig, AprilTagPoseEstimator.Config poseEstConfig,
        TrcDbgTrace tracer)
    {
        this.tagFamily = tagFamily;
        this.tracer = tracer;
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
            poseEstimator = new AprilTagPoseEstimator(poseEstConfig);
        }
        else
        {
            poseEstimator = null;
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

    //
    // Implements TrcOpenCvPipeline interface.
    //

    /**
     * This method is called to reset the state of the pipeline if any.
     */
    @Override
    public void reset()
    {
        performanceMetrics.reset();
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
        final String funcName = "process";
        DetectedObject[] detectedObjects;
        double startTime = TrcTimer.getCurrentTime();

        intermediateMats[0] = input;
        // Convert to grayscale.
        Imgproc.cvtColor(input, grayMat, Imgproc.COLOR_BGR2GRAY);
        AprilTagDetection[] detections = aprilTagDetector.detect(grayMat);
        performanceMetrics.logProcessingTime(startTime);
        performanceMetrics.printMetrics(tracer);

        detectedObjects = new DetectedObject[detections.length];
        for (int i = 0; i < detectedObjects.length; i++)
        {
            detectedObjects[i] = new DetectedObject(detections[i]);
            if (tracer != null)
            {
                tracer.traceInfo(
                    funcName, "[%d: %.3f] DetectedObj=%s", i, TrcTimer.getModeElapsedTime(), detectedObjects[i]);
            }
        }

        if (annotate)
        {
            Mat output = getIntermediateOutput(intermediateStep);
            Scalar color = intermediateStep == 0? ANNOTATE_RECT_COLOR: ANNOTATE_RECT_WHITE;
            annotateFrame(output, detectedObjects, color, ANNOTATE_RECT_THICKNESS);
//            // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
//            // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
//            for (AprilTagDetection detection : detections)
//            {
//                SixDofPose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagSizeX, tagSizeY);
//                drawAxisMarker(output, tagSizeY/2.0, 3, pose.rvec, pose.tvec, cameraMatrix);
//                draw3dCubeMarker(output, tagSizeX, tagSizeX, tagSizeY, 3, pose.rvec, pose.tvec, cameraMatrix);
//                Imgproc.rectangle(output, DetectedObject.getDetectedRect(detection), GREEN, 3);
//            }
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
     * This method sets the intermediate mat of the pipeline as the video output mat and optionally annotate the
     * detected rectangle on it.
     *
     * @param intermediateStep specifies the intermediate mat used as video output (1 is the original mat, 0 to
     *        disable video output if supported).
     * @param annotate specifies true to annotate detected rectangles on the output mat, false otherwise.
     *        This parameter is ignored if intermediateStep is 0.
     */
    @Override
    public void setVideoOutput(int intermediateStep, boolean annotate)
    {
        if (intermediateStep >= 0 && intermediateStep < intermediateMats.length)
        {
            this.intermediateStep = intermediateStep;
            this.annotate = annotate;
        }
    }   //setVideoOutput

    /**
     * This method cycles to the next intermediate mat of the pipeline as the video output mat.
     *
     * @param annotate specifies true to annotate detected rectangles on the output mat, false otherwise.
     *        This parameter is ignored if intermediateStep is 0.
     */
    @Override
    public void setNextVideoOutput(boolean annotate)
    {
        intermediateStep = (intermediateStep + 1) % intermediateMats.length;
        this.annotate = annotate;
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

    // /**
    //  * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
    //  *
    //  * @param buf the RGB buffer on which to draw the marker
    //  * @param length the length of each of the marker 'poles'
    //  * @param thickness the thickness of the lines drawn.
    //  * @param rvec the rotation vector of the detection
    //  * @param tvec the translation vector of the detection
    //  * @param cameraMatrix the camera matrix used when finding the detection
    //  */
    // private void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    // {
    //     // The points in 3D space we wish to project onto the 2D image plane.
    //     // The origin of the coordinate space is assumed to be in the center of the detection.
    //     MatOfPoint3f axis = new MatOfPoint3f(
    //         new Point3(0, 0, 0),
    //         new Point3(length,0,0),
    //         new Point3(0,length,0),
    //         new Point3(0,0,-length)
    //     );

    //     // Project those points
    //     MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
    //     Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
    //     Point[] projectedPoints = matProjectedPoints.toArray();

    //     // Draw the marker!
    //     Imgproc.line(buf, projectedPoints[0], projectedPoints[1], RED, thickness);
    //     Imgproc.line(buf, projectedPoints[0], projectedPoints[2], GREEN, thickness);
    //     Imgproc.line(buf, projectedPoints[0], projectedPoints[3], BLUE, thickness);

    //     Imgproc.circle(buf, projectedPoints[0], thickness, WHITE, -1);
    // }   //drawAxisMarker

    // /**
    //  * Draw a 3D cube marker on a detection.
    //  *
    //  * @param buf the RGB buffer on which to draw the marker
    //  * @param length the cube depth.
    //  * @param tagWidth the cube width.
    //  * @param tagHeight the cube height.
    //  * @param thickness the thickness of the lines drawn.
    //  * @param rvec the rotation vector of the detection
    //  * @param tvec the translation vector of the detection
    //  * @param cameraMatrix the camera matrix used when finding the detection
    //  */
    // private void draw3dCubeMarker(
    //     Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    // {
    //     //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
    //     //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

    //     // The points in 3D space we wish to project onto the 2D image plane.
    //     // The origin of the coordinate space is assumed to be in the center of the detection.
    //     MatOfPoint3f axis = new MatOfPoint3f(
    //         new Point3(-tagWidth/2, tagHeight/2,0),
    //         new Point3( tagWidth/2, tagHeight/2,0),
    //         new Point3( tagWidth/2,-tagHeight/2,0),
    //         new Point3(-tagWidth/2,-tagHeight/2,0),
    //         new Point3(-tagWidth/2, tagHeight/2,-length),
    //         new Point3( tagWidth/2, tagHeight/2,-length),
    //         new Point3( tagWidth/2,-tagHeight/2,-length),
    //         new Point3(-tagWidth/2,-tagHeight/2,-length));

    //     // Project those points
    //     MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
    //     Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
    //     Point[] projectedPoints = matProjectedPoints.toArray();

    //     // Pillars
    //     for (int i = 0; i < 4; i++)
    //     {
    //         Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], BLUE, thickness);
    //     }

    //     // Base lines
    //     //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
    //     //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
    //     //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
    //     //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

    //     // Top lines
    //     Imgproc.line(buf, projectedPoints[4], projectedPoints[5], GREEN, thickness);
    //     Imgproc.line(buf, projectedPoints[5], projectedPoints[6], GREEN, thickness);
    //     Imgproc.line(buf, projectedPoints[6], projectedPoints[7], GREEN, thickness);
    //     Imgproc.line(buf, projectedPoints[4], projectedPoints[7], GREEN, thickness);
    // }   //draw3dCubeMarker

    // /*
    //  * A simple container to hold both rotation and translation vectors, which together form a 6DOF pose.
    //  */
    // static class SixDofPose
    // {
    //     Mat rvec;
    //     Mat tvec;

    //     public SixDofPose()
    //     {
    //         rvec = new Mat();
    //         tvec = new Mat();
    //     }   //SixDofPose

    //     public SixDofPose(Mat rvec, Mat tvec)
    //     {
    //         this.rvec = rvec;
    //         this.tvec = tvec;
    //     }
    // }   //class SixDofPose

    // /**
    //  * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
    //  * original size of the tag.
    //  *
    //  * @param points the points which form the trapezoid
    //  * @param cameraMatrix the camera intrinsics matrix
    //  * @param tagsizeX the original width of the tag
    //  * @param tagsizeY the original height of the tag
    //  * @return the 6DOF pose of the camera relative to the tag
    //  */
    // private SixDofPose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY)
    // {
    //     // The actual 2d points of the tag detected in the image
    //     MatOfPoint2f points2d = new MatOfPoint2f(points);

    //     // The 3d points of the tag in an 'ideal projection'
    //     Point3[] arrayPoints3d = new Point3[4];
    //     arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
    //     arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
    //     arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
    //     arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
    //     MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

    //     // Using this information, actually solve for pose
    //     SixDofPose pose = new SixDofPose();
    //     Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

    //     return pose;
    // }   //poseFromTrapezoid

}  //class FrcOpenCvAprilTagPipeline
