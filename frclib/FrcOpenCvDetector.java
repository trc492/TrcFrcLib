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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcOpenCvPipeline;
import TrcCommonLib.trclib.TrcVisionProcessor;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcCommonLib.trclib.TrcVisionTask;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
 
/**
 * This class implements a platform dependent OpenCV detector. It implements the TrcVisionProcessor interface which
 * provides methods to use OpenCV to grab a frame, process a frame with the provided OpenCV pipeline and put an
 * annotated frame back to the output video stream. It creates a standalone task to do the vision processing so
 * it doesn't affect the main robot thread's performance.
 */
public class FrcOpenCvDetector<O> implements TrcVisionProcessor<Mat, TrcOpenCvDetector.DetectedObject<O>>
{
    private static final Scalar ANNOTATE_COLOR = new Scalar(0,255,0,255);
    private static final int ANNOTATE_RECT_THICKNESS = 3;

    private final String instanceName;
    private final CvSource cvSource;
    private final CvSink cvSink;
    private final int imageWidth;
    private final int imageHeight;
    private final TrcDbgTrace tracer;
    private final TrcHomographyMapper homographyMapper;
    private final TrcVisionTask<Mat, TrcOpenCvDetector.DetectedObject<O>> visionTask;
    private volatile TrcOpenCvPipeline<TrcOpenCvDetector.DetectedObject<O>> openCvPipeline = null;
    private Object pipelineLock = new Object();

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numImageBuffer specifies the number of image buffers to allocate.
     * @param cvSink specifies the object to capture the video frames.
     * @param cvSource specifies the object to stream video output.
     * @param imageWidth specifies the width of the camera image.
     * @param imageHeight specifies the height of the camera image.
     * @param cameraRect specifies the camera rectangle for Homography Mapper, can be null if not provided.
     * @param worldRect specifies the world rectangle for Homography Mapper, can be null if not provided.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public FrcOpenCvDetector(
        String instanceName, int numImageBuffers, CvSink cvSink, CvSource cvSource, int imageWidth, int imageHeight,
        TrcHomographyMapper.Rectangle cameraRect,TrcHomographyMapper.Rectangle worldRect, TrcDbgTrace tracer)
    {
        this.instanceName = instanceName;
        this.cvSource = cvSource;
        this.cvSink = cvSink;
        this.imageWidth = imageWidth;
        this.imageHeight = imageHeight;
        this.tracer = tracer;

        if (cameraRect != null && worldRect != null)
        {
            homographyMapper = new TrcHomographyMapper(cameraRect, worldRect);
        }
        else
        {
            homographyMapper = null;
        }
        //
        // Pre-allocate the image buffers.
        //
        Mat[] imageBuffers = new Mat[numImageBuffers];
        for (int i = 0; i < imageBuffers.length; i++)
        {
            imageBuffers[i] = new Mat();
        }

        visionTask = new TrcVisionTask<>(instanceName, this, imageBuffers);
        visionTask.setPerfReportEnabled(tracer);
    }   //FrcOpenCvDetector

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString
    
    /**
     * This method sets the OpenCV pipeline to be used for the detection.
     *
     * @param pipeline specifies the pipeline to be used for detection.
     */
    public void setPipeline(TrcOpenCvPipeline<TrcOpenCvDetector.DetectedObject<O>> pipeline)
    {
        synchronized (pipelineLock)
        {
            if (pipeline != openCvPipeline)
            {
                // Pipeline has changed.
                // Enable vision task if setting a new pipeline, disable if setting null pipeline.
                openCvPipeline = pipeline;
                setEnabled(pipeline != null);
            }
        }
    }   //setPipeline
 
    /**
     * This method returns the current pipeline.
     *
     * @return current pipeline, null if no set pipeline.
     */
    public TrcOpenCvPipeline<TrcOpenCvDetector.DetectedObject<O>> getPipeline()
    {
        return openCvPipeline;
    }   //getPipeline
 
    /**
     * This method pauses/resumes pipeline processing.
     *
     * @param enabled specifies true to start pipeline processing, false to stop.
     */
    public void setEnabled(boolean enabled)
    {
        synchronized (pipelineLock)
        {
            boolean taskEnabled = visionTask.isTaskEnabled();

            if (enabled && !taskEnabled)
            {
                if (openCvPipeline != null)
                {
                    openCvPipeline.reset();
                    visionTask.setTaskEnabled(true);
                }
            }
            else if (!enabled && taskEnabled)
            {
                visionTask.setTaskEnabled(false);
            }
        }
    }   //setEnabled

    /**
     * This method returns the state of EocvVision.
     *
     * @return true if the EocvVision is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        return visionTask.isTaskEnabled();
    }   //isEnabled

     /**
     * This method enables/disables image annotation of the detected object rects.
     *
     * @param enabled specifies true to enable video out, false to disable.
     */
    public void setVideoOutEnabled(boolean enabled)
    {
        visionTask.setVideoOutEnabled(enabled);
    }   //setVideoOutEnabled

    /**
     * This method sets the vision task processing interval.
     *
     * @param interval specifies the processing interval in msec. If 0, process as fast as the CPU can run.
     */
    public void setProcessingInterval(long interval)
    {
        visionTask.setProcessingInterval(interval);
    }   //setProcessingInterval

    /**
     * This method returns the vision task processing interval.
     *
     * @return vision task processing interval in msec.
     */
    public long getProcessingInterval()
    {
        return visionTask.getProcessingInterval();
    }   //getProcessingInterval

   /**
     * This method returns an array of detected targets from EasyOpenCV vision.
     *
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @param objHeightOffset specifies the object height offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     * @return array of detected target info.
     */
    @SuppressWarnings("unchecked")
    public TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<O>>[] getDetectedTargetsInfo(
        TrcOpenCvDetector.FilterTarget filter,
        Comparator<? super TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<O>>> comparator,
        double objHeightOffset, double cameraHeight)
    {
        final String funcName = instanceName + ".getDetectedTargetsInfo";
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<O>>[] detectedTargets = null;
        TrcOpenCvDetector.DetectedObject<O>[] objects = visionTask.getDetectedObjects();

        if (objects != null)
        {
            ArrayList<TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<O>>> targetList = new ArrayList<>();

            for (TrcOpenCvDetector.DetectedObject<O> obj : objects)
            {
                if (filter == null || filter.validateTarget(obj))
                {
                    TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<O>> targetInfo =
                        new TrcVisionTargetInfo<>(
                            obj, imageWidth, imageHeight, homographyMapper, objHeightOffset, cameraHeight);
                    targetList.add(targetInfo);
                }
            }

            if (targetList.size() > 0)
            {
                detectedTargets = targetList.toArray(new TrcVisionTargetInfo[0]);
                if (comparator != null && detectedTargets.length > 1)
                {
                    Arrays.sort(detectedTargets, comparator);
                }
            }

            if (detectedTargets != null && tracer != null)
            {
                for (int i = 0; i < detectedTargets.length; i++)
                {
                    tracer.traceInfo(funcName, "[%d] Target=%s", i, detectedTargets[i]);
                }
            }
        }

        return detectedTargets;
    }   //getDetectedTargetsInfo
 
    //
    // Implements TrcVisionProcess interface.
    //

    /**
     * This method takes a snapshot of the video frame.
     *
     * @param frame specifies the frame buffer to hold the video snapshot.
     * @return true if successful, false otherwise.
     */
    public boolean getFrame(Mat frame)
    {
        return cvSink.grabFrame(frame) > 0;
    }   //getFrame

    /**
     * This method displays a frame buffer to the display surface.
     *
     * @param frame specifies the video frame to be displayed.
     */
    public void putFrame(Mat frame)
    {
        cvSource.putFrame(frame);
    }   //putFrame

    /**
     * This method is called to detect objects in the acquired image frame.
     *
     * @param image specifies the image to be processed.
     * @return detected objects, null if none detected.
     */
    public TrcOpenCvDetector.DetectedObject<O>[] processFrame(Mat image)
    {
        openCvPipeline.process(image);
        return openCvPipeline.getDetectedObjects();
    }   //processFrame

    /**
     * This method is called to overlay rectangles of the detected objects on an image.
     *
     * @param image specifies the frame to be rendered to the video output.
     * @param detectedObjects specifies the detected objects.`
     */
    public void annotateFrame(Mat image, TrcOpenCvDetector.DetectedObject<O>[] detectedObjects)
    {
        for (TrcOpenCvDetector.DetectedObject<O> object : detectedObjects)
        {
            Rect rect = object.getRect();
            Imgproc.rectangle(image, rect, ANNOTATE_COLOR, ANNOTATE_RECT_THICKNESS);
        }
    }   //annotateFrame
   
}   //class FrcOpenCvDetector
 