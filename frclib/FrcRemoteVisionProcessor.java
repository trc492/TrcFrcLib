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
    /**
     * Process the latest data from network tables.
     *
     * @return The relative pose of the object being tracked. null if no object detected.
     */
    protected abstract RelativePose processData();

    private final List<RelativePose> frames = new LinkedList<>();
    protected final TrcDbgTrace tracer;
    protected final String instanceName;
    protected final NetworkTable networkTable;
    private final TrcTaskMgr.TaskObject visionTaskObj;
    private final Relay ringLight;

    private int maxCachedFrames = 10; // the last 10 frames
    private double offsetX = 0.0;
    private double offsetY = 0.0;
    private double timeout = 0.0;
    private volatile RelativePose relativePose = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param networkTableName specifies the network table name.
     */
    public FrcRemoteVisionProcessor(String instanceName, String networkTableName, int relayPort)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        networkTable = instance.getTable(networkTableName);
        instance.addConnectionListener(false, this::connectionListener);
        visionTaskObj = TrcTaskMgr.createTask(instanceName + ".visionTask", this::updateTargetInfo);

        if (relayPort >= 0)
        {
            ringLight = new Relay(relayPort);
            ringLight.setDirection(Relay.Direction.kForward);
        }
        else
        {
            ringLight = null;
        }
    }   //FrcRemoteVisionProcessor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param networkTableName specifies the network table name.
     */
    public FrcRemoteVisionProcessor(String instanceName, String networkTableName)
    {
        this(instanceName, networkTableName, -1);
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
     * This method is called when a network table event happened. It monitors the network table connection.
     *
     * @param event specifies the network table event.
     */
    private void connectionListener(NetworkTableEvent event)
    {
        tracer.traceDebug(
            instanceName,
            "TableEvent(remoteIp=" + event.connInfo.remote_ip + ", connected=" + event.is(Kind.kConnected) + ")");
    }   //connectionListener

    /**
     * This method enables or disables the remote vision processor. The ring light is also enabled or disabled
     * accordingly.
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
     * This method turns the right light ON or OFF.
     *
     * @param enabled specifies true to turn ring light ON, false to turn OFF.
     */
    public void setRingLightEnabled(boolean enabled)
    {
        if (ringLight != null)
        {
            ringLight.set(enabled ? Relay.Value.kOn : Relay.Value.kOff);
        }
    }   //setRingLightEnabled

    /**
     * This method sets the offset of the camera. This is the distance in the x and y axes from the camera to the
     * robot perspective. Positive is forward and right.
     *
     * @param x X component of distance to camera.
     * @param y Y component of distance to camera.
     */
    public void setOffsets(double x, double y)
    {
        this.offsetX = x;
        this.offsetY = y;
    }   //setOffsets

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
     * This method is run periodically to perform the vision task which is to update the target info.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the current robot run mode.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void updateTargetInfo(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        // Deserialize the latest calculated pose
        RelativePose relPose = processData();

        synchronized(frames)
        {
            if (relPose == null)
            {
                // We have not found a pose, so set to null
                // Clear the frames list if it is not empty
                frames.clear();
            }
            else
            {
                // Adjust for the camera offset and recalculate polar coordinates
                relPose.x += offsetX;
                relPose.y += offsetY;
                relPose.recalculatePolarCoords();
                // Add the latest pose
                frames.add(relPose);
                // Trim the list so only the last few are kept
                while (frames.size() > maxCachedFrames)
                {
                    frames.remove(0);
                }
            }
            this.relativePose = relPose;
        }
    }   //updateTargetInfo

    /**
     * This method checks if a relative pose is still fresh by checking its timestamp against the timeout.
     *
     * @param relPose specifies the relative pose to check.
     * @return true if the timestamp of relPose is still within timeout, false otherwise.
     */
    private boolean isFresh(RelativePose relPose)
    {
        return relPose != null && (timeout == 0.0 || TrcTimer.getCurrentTime() - relPose.time <= timeout);
    }   //isFresh

    /**
     * This method retrieves the value associated with the given network table key.
     *
     * @param key specifies the network table key.
     * @return value for the given key.
     */
    public double get(String key)
    {
        return networkTable.getEntry(key).getDouble(0.0);
    }   //get

    /**
     * This method retrieves the network table value associated with the given network table key.
     *
     * @param key specifies the network table key.
     * @return network table value for the given key.
     */
    public NetworkTableValue getValue(String key)
    {
        return networkTable.getEntry(key).getValue();
    }   //getValue

    /**
     * Calculates the average pose of the last numFrames frames, optionally requiring numFrames frames.
     *
     * @param numFrames  How many frames to average.
     * @param requireAll If true, require at least numFrames frames to average.
     * @return Average of last numFrames frames, or null if not enough frames and requireAll is true.
     */
    public RelativePose getAveragePose(int numFrames, boolean requireAll)
    {
        RelativePose avgPose = null;

        synchronized (frames)
        {
            if (!frames.isEmpty() && (!requireAll || numFrames <= frames.size()))
            {
                int fromIndex = Math.max(0, frames.size() - numFrames);
                List<RelativePose> poses = frames.subList(fromIndex, frames.size());
                int numFreshFrames = 0;

                avgPose = new RelativePose();
                for (RelativePose pose : poses)
                {
                    // Only use data if it's fresh.
                    if (isFresh(pose))
                    {
                        avgPose.objectYaw += pose.objectYaw;
                        avgPose.r += pose.r;
                        avgPose.theta += pose.theta;
                        avgPose.x += pose.x;
                        avgPose.y += pose.y;
                        numFreshFrames++;
                    }
                }

                if (numFreshFrames > 0)
                {
                    avgPose.objectYaw /= (double) numFreshFrames;
                    avgPose.r /= (double) numFreshFrames;
                    avgPose.theta /= (double) numFreshFrames;
                    avgPose.x /= (double) numFreshFrames;
                    avgPose.y /= (double) numFreshFrames;
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
    public RelativePose getAveragePose(int numFrames)
    {
        return getAveragePose(numFrames, false);
    }   //getAveragePose

    /**
     * Get the average pose of the last n frames where n=maxCachedFrames.
     *
     * @return The average pose. (all attributes averaged)
     */
    public RelativePose getAveragePose()
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
    public RelativePose getMedianPose(int numFrames, boolean requireAll)
    {
        RelativePose median = null;

        synchronized (frames)
        {
            if (!frames.isEmpty() && (!requireAll || numFrames <= frames.size()))
            {
                int fromIndex = Math.max(0, frames.size() - numFrames);
                List<RelativePose> poses = frames.subList(fromIndex, frames.size());

                poses = poses.stream().filter(this::isFresh).collect(Collectors.toList());
                if (!poses.isEmpty())
                {
                    median = new RelativePose();
                    median.x = TrcUtil.median(frames.stream().mapToDouble(e -> e.x).toArray());
                    median.y = TrcUtil.median(frames.stream().mapToDouble(e -> e.y).toArray());
                    median.r = TrcUtil.median(frames.stream().mapToDouble(e -> e.r).toArray());
                    median.theta = TrcUtil.median(frames.stream().mapToDouble(e -> e.theta).toArray());
                    median.objectYaw = TrcUtil.median(frames.stream().mapToDouble(e -> e.objectYaw).toArray());
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

    /**
     * This class encapsulate the info representing a relative position with a timestamp.
     */
    public static class RelativePose
    {
        public double r, theta, objectYaw, x, y;
        public double time;

        /**
         * This method is called to update the polar coordinates of the relative pose.
         */
        public void recalculatePolarCoords()
        {
            r = TrcUtil.magnitude(x, y);
            theta = Math.toDegrees(Math.atan2(x, y));
        }   //recalculatePolarCoords

        /**
         * This method returns the relative pose info in string format.
         */
        @Override
        public String toString()
        {
            return "RelativePose(x=" + x + ",y=" + y + ",r=" + r + ",theta=" + theta + ")";
        }   //toString

    }   //class RelativePose

}   //class FrcRemoteVisionProcessor
