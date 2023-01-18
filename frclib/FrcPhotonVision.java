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

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcVisionTargetInfo;

/**
 * This class implements vision detection using PhotonLib.
 */
public class FrcPhotonVision
{
    /**
     * This class encapsulates info of the detected object. It extends TrcOpenCvDetector.DetectedObject that requires
     * it to provide a method to return the detected object rect and area.
     */
    public static class DetectedObject implements TrcVisionTargetInfo.ObjectInfo
    {
        public PhotonTrackedTarget target;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param targeet specifies the photon detected target.
         */
        public DetectedObject(PhotonTrackedTarget target)
        {
            this.target = target;
        }   //DetectedObject

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @Override
        public String toString()
        {
            return "(" + target + ",Rect=" + getRect() + ",Area=" + getArea() + ")";
        }   //toString

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getRect()
        {
            List<TargetCorner> corners = target.getDetectedCorners();
            TargetCorner lowerLeftCorner = corners.get(0);
            TargetCorner lowerRightCorner = corners.get(1);
            TargetCorner upperRightCorner = corners.get(2);
            TargetCorner upperLeftCorner = corners.get(3);
            double width = ((upperRightCorner.x - upperLeftCorner.x) + (lowerRightCorner.x - lowerLeftCorner.x))/2.0;
            double height = ((lowerLeftCorner.y - upperLeftCorner.y) + (lowerRightCorner.y - upperRightCorner.y))/2.0;

            return new Rect((int)upperLeftCorner.x, (int)upperLeftCorner.y, (int)width, (int)height);
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

    public final PhotonCamera camera;
    private final TrcDbgTrace tracer;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraName specifies the photon vision camera name.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public FrcPhotonVision(String cameraName, TrcDbgTrace tracer)
    {
        camera = new PhotonCamera(cameraName);
        this.tracer = tracer;
    }   //FrcPhotonVision

    /**
     * This method returns the photon camera name.
     *
     * @return photon camera name.
     */
    @Override
    public String toString()
    {
        return camera.getName();
    }   //toString

    /**
     * This method returns the array of detected objects.
     *
     * @return array of detected objects.
     */
    public DetectedObject[] getDetectedObjects()
    {
        final String funcName = "getDetectedObjects";
        DetectedObject[] detectedObjs = null;
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets())
        {
            List<PhotonTrackedTarget> targets = result.getTargets();
            detectedObjs = new DetectedObject[targets.size()];

            for (int i = 0; i < targets.size(); i++)
            {
                detectedObjs[i] = new DetectedObject(targets.get(i));
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
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets())
        {
            PhotonTrackedTarget target = result.getBestTarget();
            bestDetectedObj = new DetectedObject(target);
            if (tracer != null)
            {
                tracer.traceInfo(
                    funcName, "[%.3f] DetectedObj=%s", TrcTimer.getModeElapsedTime(), bestDetectedObj);
            }
        }

        return bestDetectedObj;
    }   //getBestDetectedObject

    /**
     * This method returns the latest photon pipeline detection result.
     *
     * @return latest pipeline result.
     */
    public PhotonPipelineResult getLatestResult()
    {
        return camera.getLatestResult();
    }   //getLatestResult

    /**
     * This method checks if photon has the latest detected targets.
     *
     * @return true if has the latest targets, false otherwise.
     */
    public boolean hasLatestTarget()
    {
        return camera.getLatestResult().hasTargets();
    }   //hasLatestTarget

    /**
     * This method returns a list of the latest detected targets.
     *
     * @return list of the latest detected targets.
     */
    public List<PhotonTrackedTarget> getLatestTargets()
    {
        return camera.getLatestResult().getTargets();
    }   //getLatestTargets

    /**
     * This method returns the best of the latest detected targets.
     *
     * @return best of the latest detected targets.
     */
    public PhotonTrackedTarget getLatestBestTarget()
    {
        return camera.getLatestResult().getBestTarget();
    }   //getLatestBestTarget

}   //class PhotonVision
