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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.Relay;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;

import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;

public abstract class FrcRemoteVisionProcessor
{
    private final String instanceName;
    protected final NetworkTable networkTable;

    private volatile RelativePose relativePose = null;
    private int maxCachedFrames = 10; // the last 10 frames
    private List<RelativePose> frames = new LinkedList<>();
    private final Object framesLock = new Object();
    private Relay ringLight;
    private double timeout = 0.0;
    private double offsetX = 0.0;
    private double offsetY = 0.0;
    private TrcTaskMgr.TaskObject visionTaskObj;

    public FrcRemoteVisionProcessor(String instanceName, String networkTableName)
    {
        this.instanceName = instanceName;
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        networkTable = instance.getTable(networkTableName);
        instance.addConnectionListener(false, this::connectionListener);
        visionTaskObj = TrcTaskMgr.createTask(instanceName + ".visionTask", this::updateTargetInfo);
    }   //FrcRemoteVisionProcessor

    public FrcRemoteVisionProcessor(String instanceName, String networkTableName, int relayPort)
    {
        this(instanceName, networkTableName);
        ringLight = new Relay(relayPort);
        ringLight.setDirection(Relay.Direction.kForward);
    }   //FrcRemoteVisionProcessor

    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * Enables or disables the remote vision processor. The ring light is also enabled or disabled accordingly.
     *
     * @param enabled If true, enable the ring light and processor. Disable both otherwise.
     */
    public void setEnabled(boolean enabled)
    {
        setRingLightEnabled(enabled);
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
     * Set the offset of the camera. This is the distance in the x and y axes from the camera to the robot perspective.
     * Positive is forward and right.
     *
     * @param x X component of distance to camera.
     * @param y Y component of distance to camera.
     */
    public void setOffsets(double x, double y)
    {
        this.offsetX = x;
        this.offsetY = y;
    }   //setOffsets

    public void setFreshnessTimeout(double timeout)
    {
        this.timeout = timeout;
    }   //setFreshnessTimeout

    public void setRingLightEnabled(boolean enabled)
    {
        if (ringLight != null)
        {
            ringLight.set(enabled ? Relay.Value.kOn : Relay.Value.kOff);
        }
    }   //setRingLightEnabled

    private void connectionListener(NetworkTableEvent event)
    {
        TrcDbgTrace.getGlobalTracer().traceInfo(
            "connectionListener", "TableEvent(remoteIp=%s, connected=%s",
            event.connInfo.remote_ip, event.is(Kind.kConnected));
    }   //connectionListener

    /**
     * Process the latest data from network tables.
     *
     * @return The relative pose of the object being tracked. null if no object detected.
     */
    protected abstract RelativePose processData();

    private void updateTargetInfo(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        // Deserialize the latest calculated pose
        RelativePose relativePose = processData();
        if (relativePose == null)
        {
            // We have not found a pose, so set to null
            this.relativePose = null;
            synchronized (framesLock)
            {
                // Clear the frames list if it is not empty
                if (!frames.isEmpty())
                {
                    frames.clear();
                }
            }
        }
        else
        {
            // Adjust for the camera offset and recalculate polar coordinates
            relativePose.x += offsetX;
            relativePose.y += offsetY;
            relativePose.recalculatePolarCoords();
            this.relativePose = relativePose;
            synchronized (framesLock)
            {
                // Add the latest pose
                frames.add(relativePose);
                // Trim the list so only the last few are kept
                while (frames.size() > maxCachedFrames)
                {
                    frames.remove(0);
                }
            }
        }
    }   //updateTargetInfo

    private boolean isFresh(RelativePose pose)
    {
        return pose != null && (timeout == 0.0 || TrcTimer.getCurrentTime() - pose.time <= timeout);
    }   //isFresh

    public double get(String key)
    {
        return networkTable.getEntry(key).getDouble(0.0);
    }   //get

    public NetworkTableValue getValue(String key)
    {
        return networkTable.getEntry(key).getValue();
    }   //getValue

    /**
     * Get the average pose of the last n frames where n=maxCachedFrames.
     *
     * @return The average pose. (all attributes averaged)
     */
    public RelativePose getAveragePose()
    {
        return getAveragePose(maxCachedFrames);
    }   //getAveragePose

    /**
     * Calculates the average pose of the last numFrames frames.
     *
     * @param numFrames How many frames to average.
     * @return The average pose.
     */
    public RelativePose getAveragePose(int numFrames)
    {
        return getAveragePose(numFrames, false);
    }   //getAveragePose

    /**
     * Calculates the average pose of the last numFrames frames, optionally requiring numFrames frames.
     *
     * @param numFrames  How many frames to average.
     * @param requireAll If true, require at least numFrames frames to average.
     * @return Average of last numFrames frames, or null if not enough frames and requireAll is true.
     */
    public RelativePose getAveragePose(int numFrames, boolean requireAll)
    {
        RelativePose average = new RelativePose();
        synchronized (framesLock)
        {
            if ((requireAll && frames.size() < numFrames) || frames.isEmpty())
            {
                return null;
            }
            int fromIndex = Math.max(0, frames.size() - numFrames);
            List<RelativePose> poses = frames.subList(fromIndex, frames.size());
            int numFreshFrames = 0;
            for (RelativePose pose : poses)
            {
                // Only use data if it's fresh
                if (isFresh(pose))
                {
                    average.objectYaw += pose.objectYaw;
                    average.r += pose.r;
                    average.theta += pose.theta;
                    average.x += pose.x;
                    average.y += pose.y;
                    numFreshFrames++;
                }
            }
            // If no fresh data, clear all and return null
            if (numFreshFrames == 0)
            {
                frames.clear();
                return null;
            }
            average.objectYaw /= (double) numFreshFrames;
            average.r /= (double) numFreshFrames;
            average.theta /= (double) numFreshFrames;
            average.x /= (double) numFreshFrames;
            average.y /= (double) numFreshFrames;
        }
        return average;
    }   //getAveragePose

    /**
     * Calculates the median pose of the last numFrames frames, optionally requiring numFrames frames.
     *
     * @param numFrames  How many frames to calculate with.
     * @param requireAll If true, require at least numFrames frames.
     * @return Median of last numFrames frames, or null if not enough frames and requireAll is true, or if all data
     *         is stale.
     */
    public RelativePose getMedianPose(int numFrames, boolean requireAll)
    {
        RelativePose median = new RelativePose();
        synchronized (framesLock)
        {
            if ((requireAll && frames.size() < numFrames) || frames.isEmpty())
            {
                return null;
            }
            int fromIndex = Math.max(0, frames.size() - numFrames);
            List<RelativePose> poses = frames.subList(fromIndex, frames.size());
            poses = poses.stream().filter(this::isFresh).collect(Collectors.toList());
            if (poses.isEmpty())
            {
                frames.clear();
                return null;
            }

            median.x = TrcUtil.median(frames.stream().mapToDouble(e -> e.x).toArray());
            median.y = TrcUtil.median(frames.stream().mapToDouble(e -> e.y).toArray());
            median.r = TrcUtil.median(frames.stream().mapToDouble(e -> e.r).toArray());
            median.theta = TrcUtil.median(frames.stream().mapToDouble(e -> e.theta).toArray());
            median.objectYaw = TrcUtil.median(frames.stream().mapToDouble(e -> e.objectYaw).toArray());
        }
        return median;
    }   //getMedianPose

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
    }   //getMaxCachedFrames

    /**
     * Gets the last calculated pose.
     *
     * @return The pose calculated by the vision system. If no object detected, returns null.
     */
    public RelativePose getLastPose()
    {
        RelativePose pose = relativePose;
        return isFresh(pose) ? pose : null;
    }   //getLastPose

    public static class RelativePose
    {
        public double r, theta, objectYaw, x, y;
        public double time;

        public void recalculatePolarCoords()
        {
            r = TrcUtil.magnitude(x, y);
            theta = Math.toDegrees(Math.atan2(x, y));
        }

        @Override
        public String toString()
        {
            return String.format("RelativePose(x=%.1f,y=%.1f,r=%.1f,theta=%.1f)", x, y, r, theta);
        }
    }   //class RelativePose

}   //class FrcRemoteVisionProcessor
