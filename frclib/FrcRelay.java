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

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcExclusiveSubsystem;
import TrcCommonLib.trclib.TrcTimer;
import edu.wpi.first.wpilibj.Relay;

/**
 * This class implements an FRC relay object extending Relay. It adds the capability of delay firing the relay and
 * also allows firing with a duration at which time, it will automatically turns it off.
 */
public class FrcRelay extends Relay implements TrcExclusiveSubsystem
{
    private static class TaskParams
    {
        Value value = Value.kOff;
        double duration = 0.0;
        TrcEvent completionEvent = null;
        boolean taskActive = false;
    }   //class TaskParams

    private final TaskParams taskParams = new TaskParams();
    private boolean init = false;
    private String instanceName;
    private TrcTimer timer;
    private boolean inverted = false;
    private Value prevValue = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel specifies the relay channel.
     * @param direction specifies the relay direction modes.
     */
    public FrcRelay(String instanceName, int channel, Direction direction)
    {
        super(channel, direction);
        this.instanceName = instanceName;
        timer = new TrcTimer(instanceName);
        init = true;
    }   //FrcRelay

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This methods sets the relay to inverted mode. When the relay is inverted, setting it OFF will set it to the
     * previous ON mode (could be kOn, kForward or kReverse) and setting it to any ON mode will set it to OFF.
     *
     * @param inverted specifies true to invert the channels, false otherwise.
     */
    public void setInverted(boolean inverted)
    {
        this.inverted = inverted;
        prevValue = null;
    }   //setInverted

    /**
     * This method checks if the relay is in inverted mode.
     *
     * @return true if relay is inverted, false otherwise.
     */
    public boolean isInverted()
    {
        return inverted;
    }   //isInvereted

    /**
     * This method will observe if the relay is in inverted mode and will act accordingly.
     *
     * @param value specifies the value to be set. This will check if the relay is inverted and will change the value
     *        accordingly (kOff -> previous ON mode, any ON mode -> kOff).
     * @param cancelPrevious specifies true to cancel previous operation, false otherwise.
     */
    private void set(Value value, boolean cancelPrevious)
    {
        if (!init)
        {
            super.set(value);
        }
        else
        {
            if (cancelPrevious)
            {
                cancel();
            }

            if (inverted)
            {
                if (value != Value.kOff)
                {
                    // Inverted of any of ON mode is OFF.
                    super.set(Value.kOff);
                }
                else if (prevValue != null)
                {
                    // Inverted of OFF mode is the previous ON mode.
                    // If we don't have prevValue, this is the first time setting it to inverted OFF which is unknown,
                    // so do nothing.
                    super.set(prevValue);
                }
                // Remember the intended value so we can use it next time when setting to kOff.
                prevValue = value;
            }
            else
            {
                // Not inverted, set it as-is.
                super.set(value);
            }
        }
    }   //set

    /**
     * This method overrides the set method in the Relay class. It will observe if the relay is in inverted mode and
     * will act accordingly.
     *
     * @param value specifies the value to be set. This will check if the relay is inverted and will change the value
     *        accordingly (kOff -> previous ON mode, any ON mode -> kOff).
     */
    @Override
    public void set(Value value)
    {
        set(value, true);
    }   //set

    /**
     * This method overrides the get method in the Relay class. It will observe if the relay is in inverted mode and
     * will return the appropriate value accordingly.
     */
    @Override
    public Value get()
    {
        Value value = super.get();

        if (inverted)
        {
            value = value == Value.kOff ? prevValue : Value.kOff;
        }

        return value;
    }   //get

    /**
     * This method cancels a pending operation if any.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     */
    public void cancel(String owner)
    {
        if (validateOwnership(owner))
        {
            synchronized (taskParams)
            {
                if (taskParams.taskActive)
                {
                    timer.cancel();
                    if (taskParams.completionEvent != null)
                    {
                        taskParams.completionEvent.cancel();
                        taskParams.completionEvent = null;
                    }
                    taskParams.taskActive = false;
                }
            }
        }
    }   //cancel
 
    /**
     * This method cancels a pending operation if any.
     */
    public void cancel()
    {
        cancel(null);
    }   //cacnel

    /**
     * This method is a timer callback when the duration has expired.
     *
     * @param context specifies the TaskParams object.
     */
    private void durationExpired(Object context)
    {
        TaskParams taskParams = (TaskParams) context;

        synchronized (taskParams)
        {
            set(Value.kOff, false);
            if (taskParams.completionEvent != null)
            {
                taskParams.completionEvent.signal();
                taskParams.completionEvent = null;
            }
            taskParams.taskActive = false;
        }
    }   //durationExpired

    /**
     * This method is a timer callback when delay has expired.
     *
     * @param context specifies the TaskParams object.
     */
    private void delayExpired(Object context)
    {
        TaskParams taskParams = (TaskParams) context;

        synchronized (taskParams)
        {
            set(taskParams.value, false);
            if (taskParams.duration > 0.0)
            {
                timer.set(taskParams.duration, this::durationExpired, taskParams);
            }
            else
            {
                if (taskParams.completionEvent != null)
                {
                    taskParams.completionEvent.signal();
                    taskParams.completionEvent = null;
                }
                taskParams.taskActive = false;
            }
        }
    }   //delayExpired

    /**
     * This method sets the relay to the specified value mode with optional delay and duration.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param delay specifies delay time in seconds, zero if no delay.
     * @param value specifies the relay value mode to set it to.
     * @param duration specifies the duration in seconds, zero if no duration. If specified, it will turn the relay
     *        to kOff after duration has expired.
     * @param completionEvent specifies the event to signal when duration has expired, can be null if not provided.
     *        If duration is zero, the event will be signaled immediately after setting the value.
     */
    public void set(String owner, double delay, Value value, double duration, TrcEvent completionEvent)
    {
        TrcEvent releaseOwnershipEvent = acquireOwnership(owner, completionEvent, null);
        if (releaseOwnershipEvent != null) completionEvent = releaseOwnershipEvent;

        if (validateOwnership(owner))
        {
            synchronized (taskParams)
            {
                // Cancel previous operation if any.
                cancel();
                taskParams.value = value;
                taskParams.duration = duration;
                taskParams.completionEvent = completionEvent;
                taskParams.taskActive = true;
            }

            if (delay > 0.0)
            {
                timer.set(delay, this::delayExpired, taskParams);
            }
            else
            {
                delayExpired(taskParams);
            }
        }
    }   //set

    /**
     * This method sets the relay to the specified value mode with optional delay and duration.
     *
     * @param delay specifies delay time in seconds, zero if no delay.
     * @param value specifies the relay value mode to set it to.
     * @param duration specifies the duration in seconds, zero if no duration. If specified, it will turn the relay
     *        to kOff after duration has expired.
     * @param completionEvent specifies the event to signal when duration has expired, can be null if not provided.
     *        If duration is zero, the event will be signaled immediately after setting the value.
     */
    public void set(double delay, Value value, double duration, TrcEvent completionEvent)
    {
        set(null, delay, value, duration, completionEvent);
    }   //set

    /**
     * This method sets the relay to the specified value mode with optional delay and duration.
     *
     * @param value specifies the relay value mode to set it to.
     * @param duration specifies the duration in seconds, zero if no duration. If specified, it will turn the relay
     *        to kOff after duration has expired.
     * @param completionEvent specifies the event to signal when duration has expired, can be null if not provided.
     *        If duration is zero, the event will be signaled immediately after setting the value.
     */
    public void set(Value value, double duration, TrcEvent completionEvent)
    {
        set(null, 0.0, value, duration, completionEvent);
    }   //set

    /**
     * This method sets the relay to the specified value mode with optional delay and duration.
     *
     * @param delay specifies delay time in seconds, zero if no delay.
     * @param value specifies the relay value mode to set it to.
     * @param duration specifies the duration in seconds, zero if no duration. If specified, it will turn the relay
     *        to kOff after duration has expired.
     */
    public void set(double delay, Value value, double duration)
    {
        set(null, delay, value, duration, null);
    }   //set

    /**
     * This method sets the relay to the specified value mode with optional delay and duration.
     *
     * @param value specifies the relay value mode to set it to.
     * @param duration specifies the duration in seconds, zero if no duration. If specified, it will turn the relay
     *        to kOff after duration has expired.
     */
    public void set(Value value, double duration)
    {
        set(null, 0.0, value, duration, null);
    }   //set

}   //class FrcRelay
