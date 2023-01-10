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

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcHomographyMapper;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
 
/**
 * This class implements a platform dependent OpenCV detector. It implements the TrcVisionProcessor interface which
 * provides methods to use platform specific OpenCV to grab a frame, and output annotated frame to the output video
 * stream.
 */
public class FrcOpenCvDetector extends TrcOpenCvDetector
{
    private final CvSource cvSource;
    private final CvSink cvSink;

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
        super(instanceName, numImageBuffers, imageWidth, imageHeight, cameraRect, worldRect, tracer);
        this.cvSource = cvSource;
        this.cvSink = cvSink;
    }   //FrcOpenCvDetector

    //
    // Implements TrcVisionProcess interface.
    //

    /**
     * This method takes a snapshot of the video frame.
     *
     * @param frame specifies the frame buffer to hold the video snapshot.
     * @return true if successful, false otherwise.
     */
    @Override
    public boolean getFrame(Mat frame)
    {
        return cvSink.grabFrame(frame) > 0;
    }   //getFrame

    /**
     * This method displays a frame buffer to the display surface.
     *
     * @param frame specifies the video frame to be displayed.
     */
    @Override
    public void putFrame(Mat frame)
    {
        cvSource.putFrame(frame);
    }   //putFrame

}   //class FrcOpenCvDetector
