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

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcPose3D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;

import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

public abstract class FrcRemoteVisionProcessor
{
    /**
     * This class encapsulate the info representing the detected target pose with a timestamp.
     */
    public static class TargetData<D>
    {
        public double timestamp;
        public D data;

        /**
         * This method returns the relative pose info in string format.
         */
        @Override
        public String toString()
        {
            return "TargetData(timestamp=" + timestamp + ",targetData=" + data + ")";
        }   //toString

    }   //class TargetData

    /**
     * This method gets the latest target data from vision processor.
     *
     * @return target data of the detected object, null if no object detected.
     */
    protected abstract TargetData<?> getTargetData();

    /**
     * This method gets the target pose from the detected target data.
     *
     * @param targetData specifies the target data from which to get the target pose.
     * @return detected target pose.
     */
    protected abstract TrcPose3D getTargetPose(TargetData<?> targetData);

    private final List<TargetData<?>> frames = new LinkedList<>();
    private final AtomicReference<TargetData<?>> targetData = new AtomicReference<>();

    protected final TrcDbgTrace tracer;
    protected final String instanceName;
    private final TrcTaskMgr.TaskObject visionTaskObj;

    private double timeout = 0.0;
    private int maxCachedFrames = 10;   // the last 10 frames

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FrcRemoteVisionProcessor(String instanceName)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        visionTaskObj = TrcTaskMgr.createTask(instanceName + ".visionProcessorTask", this::visionProcessorTask);
    }   //FrcRemoteVisionProcessor

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
     * This method enables/disables the vision processor task.
     *
     * @param enabled specifies true to enable vision process task, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            visionTaskObj.registerTask(TrcTaskMgr.TaskType.INPUT_TASK);
        }
        else
        {
            visionTaskObj.unregisterTask();
        }
    }   //setEnabled

    /**
     * This method sets the timeout of a relative pose.
     *
     * @param timeout specifies relative pose timeout in seconds.
     */
    public void setFreshnessTimeout(double timeout)
    {
        this.timeout = timeout;
    }   //setFreshnessTimeout

    /**
     * Set the max number of frames to keep. You will not be able to average more frames than this.
     *
     * @param numFrames How many frames to keep for averaging at maximum.
     */
    public void setMaxCachedFrames(int numFrames)
    {
        if (numFrames < 0)
        {
            throw new IllegalArgumentException("numFrames must be >= 0!");
        }
        this.maxCachedFrames = numFrames;
    }   //setMaxCachedFrames

    /**
     * This method checks if a relative pose is still fresh by checking its timestamp against the timeout.
     *
     * @param targetData specifies the target data to check its freshness.
     * @return true if the timestamp of targetData is still within timeout, false otherwise.
     */
    private boolean isFresh(TargetData<?> targetData)
    {
        return targetData != null && (timeout == 0.0 || TrcTimer.getCurrentTime() <= targetData.timestamp + timeout);
    }   //isFresh

    /**
     * Gets the last calculated pose.
     *
     * @return The pose calculated by the vision system. If no object detected, returns null.
     */
    public TargetData<?> getLastTargetData()
    {
        TargetData<?> data = targetData.getAndSet(null);
        return isFresh(data) ? data : null;
    }   //getLastTargetData

    /**
     * Calculates the average pose of the last numFrames frames, optionally requiring numFrames frames.
     *
     * @param numFrames  How many frames to average.
     * @param requireAll If true, require at least numFrames frames to average.
     * @return Average of last numFrames frames, or null if not enough frames and requireAll is true.
     */
    public TrcPose3D getAveragePose(int numFrames, boolean requireAll)
    {
        TrcPose3D avgPose = null;

        synchronized (frames)
        {
            if (!frames.isEmpty() && (!requireAll || numFrames <= frames.size()))
            {
                int fromIndex = Math.max(0, frames.size() - numFrames);
                List<TargetData<?>> targets = frames.subList(fromIndex, frames.size());
                int numFreshFrames = 0;

                avgPose = new TrcPose3D();
                for (TargetData<?> target : targets)
                {
                    // Only use data if it's fresh.
                    if (isFresh(target))
                    {
                        TrcPose3D targetPose = getTargetPose(target);
                        avgPose.x += targetPose.x;
                        avgPose.y += targetPose.y;
                        avgPose.z += targetPose.z;
                        avgPose.yaw += targetPose.yaw;
                        avgPose.pitch += targetPose.pitch;
                        avgPose.roll += targetPose.roll;
                        numFreshFrames++;
                    }
                }

                if (numFreshFrames > 0)
                {
                    avgPose.x /= numFreshFrames;
                    avgPose.y /= numFreshFrames;
                    avgPose.z /= numFreshFrames;
                    avgPose.yaw /= numFreshFrames;
                    avgPose.pitch /= numFreshFrames;
                    avgPose.roll /= numFreshFrames;
                }
                else
                {
                    // No fresh data, clear all and return null.
                    frames.clear();
                    avgPose = null;
                }
            }
        }

        return avgPose;
    }   //getAveragePose

    /**
     * Calculates the average pose of the last numFrames frames.
     *
     * @param numFrames How many frames to average.
     * @return The average pose.
     */
    public TrcPose3D getAveragePose(int numFrames)
    {
        return getAveragePose(numFrames, false);
    }   //getAveragePose

    /**
     * Get the average pose of the last n frames where n=maxCachedFrames.
     *
     * @return The average pose. (all attributes averaged)
     */
    public TrcPose3D getAveragePose()
    {
        return getAveragePose(maxCachedFrames, false);
    }   //getAveragePose

    /**
     * Calculates the median pose of the last numFrames frames, optionally requiring numFrames frames.
     *
     * @param numFrames  How many frames to calculate with.
     * @param requireAll If true, require at least numFrames frames.
     * @return Median of last numFrames frames, or null if not enough frames and requireAll is true, or if all data
     *         is stale.
     */
    public TrcPose3D getMedianPose(int numFrames, boolean requireAll)
    {
        TrcPose3D median = null;

        synchronized (frames)
        {
            if (!frames.isEmpty() && (!requireAll || numFrames <= frames.size()))
            {
                int fromIndex = Math.max(0, frames.size() - numFrames);
                List<TargetData<?>> targets = frames.subList(fromIndex, frames.size());

                targets = targets.stream().filter(this::isFresh).collect(Collectors.toList());
                if (!targets.isEmpty())
                {
                    median = new TrcPose3D();
                    median.x = TrcUtil.median(targets.stream().mapToDouble(e -> getTargetPose(e).x).toArray());
                    median.y = TrcUtil.median(targets.stream().mapToDouble(e -> getTargetPose(e).y).toArray());
                    median.z = TrcUtil.median(targets.stream().mapToDouble(e -> getTargetPose(e).z).toArray());
                    median.yaw = TrcUtil.median(frames.stream().mapToDouble(e -> getTargetPose(e).yaw).toArray());
                    median.pitch = TrcUtil.median(frames.stream().mapToDouble(e -> getTargetPose(e).pitch).toArray());
                    median.roll = TrcUtil.median(frames.stream().mapToDouble(e -> getTargetPose(e).roll).toArray());
                }
                else
                {
                    frames.clear();
                    median = null;
                }
            }
        }

        return median;
    }   //getMedianPose

    /**
     * This method is run periodically to perform the vision task which is to cache target info into frame buffer.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the current robot run mode.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void visionProcessorTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        // Deserialize the latest calculated pose
        TargetData<?> targetData = getTargetData();
        TrcDbgTrace.globalTraceInfo(instanceName, "targetData=%s", targetData);

        synchronized(frames)
        {
            if (targetData == null)
            {
                // We have not found a target, so set to null
                // Clear the frames list if it is not empty
                frames.clear();
            }
            else
            {
                // Adjust for the camera offset and recalculate polar coordinates
                frames.add(targetData);
                // Trim the list so only the last few are kept
                while (frames.size() > maxCachedFrames)
                {
                    frames.remove(0);
                }
            }
            this.targetData.set(targetData);
        }
    }   //visionProcessorTask

}   //class FrcRemoteVisionProcessor
