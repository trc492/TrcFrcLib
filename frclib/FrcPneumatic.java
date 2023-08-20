/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import TrcCommonLib.trclib.TrcTimer;

/**
 * This class implements a platform dependent pneumatic object. A pneumatic object consists of a 1 or 2 pneumatic
 * channels for a 1 or 2 valve pneumatic cylinder.
 */
public class FrcPneumatic
{
    private static final int INDEX_EXTEND = 0;
    private static final int INDEX_RETRACT = 1;
    private static final int BITMASK_EXTEND = 1 << INDEX_EXTEND;
    private static final int BITMASK_RETRACT = 1 << INDEX_RETRACT;

    private String instanceName;
    private TrcTimer timer;
    private final Solenoid[] solenoids;

    /**
     * This method is called by all variations of the constructor to do common initialization.
     *
     * @param instanceName specifies the instance name.
     */
    private void initPneumatic(String instanceName)
    {
        this.instanceName = instanceName;
        timer = new TrcTimer(instanceName);
    }   //initPneumatic

    /**
     * Constructor: Creates a 1-valve pneumatic.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the Pneumatics Control Module.
     * @param moduleType specifies the module type to use.
     * @param extendChannel specifies the pneumatic channel to extend the cylinder.
     */
    public FrcPneumatic(String instanceName, int canId, PneumaticsModuleType moduleType, int extendChannel)
    {
        initPneumatic(instanceName);
        solenoids = new Solenoid[1];
        solenoids[INDEX_EXTEND] = new Solenoid(canId, moduleType, extendChannel);
    }   //FrcPneumatic

    /**
     * Constructor: Creates a 2-valve pneumatic.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the Pneumatics Control Module.
     * @param moduleType specifies the module type to use.
     * @param extendChannel specifies the pneumatic channel to extend the cylinder.
     * @param extendChannel specifies the pneumatic channel to retract the cylinder.
     */
    public FrcPneumatic(
        String instanceName, int canId, PneumaticsModuleType moduleType, int extendChannel, int retractChannel)
    {
        initPneumatic(instanceName);
        solenoids = new Solenoid[2];
        solenoids[INDEX_EXTEND] = new Solenoid(canId, moduleType, extendChannel);
        solenoids[INDEX_RETRACT] = new Solenoid(canId, moduleType, retractChannel);
    }   //FrcPneumatic

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
     * This method sets the state of the pneumatic channels specified in a bit mask.
     *
     * @param bitMask specifies the bit mask of the pneumatic channels to be set.
     * @param on specifies true to set the channels ON and false to set them OFF.
     */
    public void set(int bitMask, boolean on)
    {
        for (int i = 0; i < solenoids.length; i++)
        {
            if (((1 << i) & bitMask) != 0)
            {
                solenoids[i].set(on);
            }
        }
    }   //set

    /**
     * This method gets the states of the pneumatic channels specified in a bit mask.
     *
     * @param bitMask specifies the channels to get the states.
     * @return channel states in a bit mask (0: inactive, 1 active).
     */
    public int get(int bitMask)
    {
        int states = 0;

        for (int i = 0; i < solenoids.length; i++)
        {
            int channelMask = 1 << i;
            if ((channelMask & bitMask) != 0 && solenoids[i].get())
            {
                states |= channelMask;
            }
        }

        return states;
    }   //get

    /**
     * This methods activates the extend channel.
     */
    public void extend()
    {
        set(BITMASK_EXTEND, true);
    }   //extend

    /**
     * This methods activates the extend channel for the specified duration and deactivates it.
     *
     * @param duration specifies the duration in seconds (from 0.01 to 2.55 sec).
     */
    public void extend(double duration)
    {
        if (duration > 0.0)
        {
            solenoids[INDEX_EXTEND].setPulseDuration(duration);
            solenoids[INDEX_EXTEND].startPulse();
        }
        else
        {
            extend();
        }
    }   //extend

    /**
     * This method is a callback to process the delay extend action when the delay timer expired.
     *
     * @param context specifies the extend duration.
     */
    private void delayExtendAction(Object context)
    {
        extend((Double) context);
    }   //delayExtendAction

    /**
     * This method delays the specified delay time and activates the extend channel for the specified duration.
     *
     * @param delay specifies the delay time in seconds.
     * @param duration specifies the activation duration in seconds (from 0.01 to 2.55 sec), set to 0.0 if no
     *        deactivation required.
     */
    public void extend(double delay, double duration)
    {
        if (delay > 0.0)
        {
            timer.set(delay, this::delayExtendAction, (Double) duration);
        }
        else
        {
            extend(duration);
        }
    }   //extend

    /**
     * This methods activates the retract channel.
     */
    public void retract()
    {
        set(BITMASK_RETRACT, true);
    }   //retract

    /**
     * This methods activates the retract channel for the specified duration and deactivates it.
     *
     * @param duration specifies the duration in seconds (from 0.01 to 2.55 sec).
     */
    public void retract(double duration)
    {
        if (duration > 0.0)
        {
            solenoids[INDEX_RETRACT].setPulseDuration(duration);
            solenoids[INDEX_RETRACT].startPulse();
        }
        else
        {
            retract();
        }
    }   //retract

    /**
     * This method is a callback to process the delay retract action when the delay timer expired.
     *
     * @param context specifies the retract duration.
     */
    private void delayRetractAction(Object context)
    {
        retract((Double) context);
    }   //delayRetractAction

    /**
     * This method delays the specified delay time and activates the retract channel for the specified duration.
     *
     * @param delay specifies the delay time in seconds.
     * @param duration specifies the activation duration in seconds (from 0.01 to 2.55 sec), set to 0.0 if no
     *        deactivation required.
     */
    public void retract(double delay, double duration)
    {
        if (delay > 0.0)
        {
            timer.set(delay, this::delayRetractAction, (Double) duration);
        }
        else
        {
            retract(duration);
        }
    }   //retract

}   //class FrcPneumatic
