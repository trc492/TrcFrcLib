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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;

/**
 * This class implements the platform dependent joystick. It provides monitoring of the joystick buttons. If the
 * caller of this class provides a button notification handler, it will call it when there are button events.
 */
public class FrcJoystick extends Joystick
{
    private static final double DEF_DEADBAND_THRESHOLD = 0.15;
    private static final double DEF_SAMPLING_PERIOD = 0.02;     //Sampling at 50Hz.
    private double samplingPeriod = DEF_SAMPLING_PERIOD;
    private double nextPeriod = 0.0;
    private double deadbandThreshold = DEF_DEADBAND_THRESHOLD;

    protected final TrcDbgTrace tracer;
    protected final String instanceName;
    private final int port;
    private final TrcTaskMgr.TaskObject buttonEventTaskObj;
    private int prevButtons;
    private FrcButtonHandler buttonHandler = null;
    private int ySign = 1;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port         specifies the joystick port ID.
     */
    public FrcJoystick(String instanceName, int port)
    {
        super(port);

        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.port = port;
        buttonEventTaskObj = TrcTaskMgr.createTask(instanceName + ".buttonEvent", this::buttonEventTask);
        prevButtons = DriverStation.getStickButtons(port);
    }   //FrcJoystick

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName      specifies the instance name.
     * @param port              specifies the joystick port ID.
     * @param deadbandThreshold specifies the deadband of the analog sticks.
     */
    public FrcJoystick(String instanceName, int port, double deadbandThreshold)
    {
        this(instanceName, port);
        this.deadbandThreshold = deadbandThreshold;
    }   //FrcJoystick

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
     * This method sets the object that will handle button events. Any previous handler set with this method will
     * no longer receive events.
     *
     * @param buttonHandler specifies the object that will handle the button events. Set to null clear previously
     *                      set handler.
     */
    public void setButtonHandler(FrcButtonHandler buttonHandler)
    {
        this.buttonHandler = buttonHandler;
        if (buttonHandler != null)
        {
            buttonEventTaskObj.registerTask(TrcTaskMgr.TaskType.PRE_PERIODIC_TASK);
        }
        else
        {
            buttonEventTaskObj.unregisterTask();
        }
    }   //setButtonHandler

    /**
     * This method returns the current button event handler.
     *
     * @return current button event handler, null if none.
     */
    public FrcButtonHandler getButtonHandler()
    {
        return buttonHandler;
    }   //getButtonHandler

    /**
     * This method sets the joystick button sampling period. By default, it is sampling at 50Hz. One could change
     * the sampling period by calling this method.
     *
     * @param period specifies the new sampling period in seconds.
     */
    public void setSamplingPeriod(double period)
    {
        samplingPeriod = period;
    }   //setSamplingPeriod

    /**
     * This method inverts the y-axis of the analog sticks.
     *
     * @param inverted specifies true if inverting the y-axis, false otherwise.
     */
    public void setYInverted(boolean inverted)
    {
        ySign = inverted ? -1 : 1;
    }   //setYInverted

    /**
     * This method returns the value of the X analog stick.
     *
     * @param squared           specifies true to apply a squared curve to the output value, false otherwise.
     * @param deadbandThreshold specifies the deadband value to apply on this stick.
     * @return adjusted value of the X analog stick.
     */
    public double getXWithDeadband(boolean squared, double deadbandThreshold)
    {
        return adjustValueWithDeadband(getX(), squared, deadbandThreshold);
    }   //getXWithDeadband

    /**
     * This method returns the value of the X analog stick.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted value of the X analog stick.
     */
    public double getXWithDeadband(boolean squared)
    {
        return getXWithDeadband(squared, deadbandThreshold);
    }   //getXWithDeadband

    /**
     * This method returns the value of the Y analog stick.
     *
     * @param squared           specifies true to apply a squared curve to the output value, false otherwise.
     * @param deadbandThreshold specifies the deadband value to apply on this stick.
     * @return adjusted value of the Y analog stick.
     */
    public double getYWithDeadband(boolean squared, double deadbandThreshold)
    {
        return adjustValueWithDeadband(ySign * getY(), squared, deadbandThreshold);
    }   //getYWithDeadband

    /**
     * This method returns the value of the Y analog stick.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted value of the Y analog stick.
     */
    public double getYWithDeadband(boolean squared)
    {
        return getYWithDeadband(squared, deadbandThreshold);
    }   //getYWithDeadband

    /**
     * This method returns the value of the Z analog stick.
     *
     * @param squared           specifies true to apply a squared curve to the output value, false otherwise.
     * @param deadbandThreshold specifies the deadband value to apply on this stick.
     * @return adjusted value of the Z analog stick.
     */
    public double getZWithDeadband(boolean squared, double deadbandThreshold)
    {
        return adjustValueWithDeadband(getZ(), squared, deadbandThreshold);
    }   //getZWithDeadband

    /**
     * This method returns the value of the Z analog stick.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted value of the Z analog stick.
     */
    public double getZWithDeadband(boolean squared)
    {
        return getZWithDeadband(squared, deadbandThreshold);
    }   //getZWithDeadband

    /**
     * This method returns the value of the Twist analog stick.
     *
     * @param squared           specifies true to apply a squared curve to the output value, false otherwise.
     * @param deadbandThreshold specifies the deadband value to apply on this stick.
     * @return adjusted value of the Twist analog stick.
     */
    public double getTwistWithDeadband(boolean squared, double deadbandThreshold)
    {
        return adjustValueWithDeadband(getTwist(), squared, deadbandThreshold);
    }   //getTwistWithDeadband

    /**
     * This method returns the value of the Twist analog stick.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted value of the Twist analog stick.
     */
    public double getTwistWithDeadband(boolean squared)
    {
        return getTwistWithDeadband(squared, deadbandThreshold);
    }   //getTwistWithDeadband

    /**
     * This method returns the value of the analog Throttle.
     *
     * @param squared           specifies true to apply a squared curve to the output value, false otherwise.
     * @param deadbandThreshold specifies the deadband value to apply on this stick.
     * @return adjusted value of the analog Throttle.
     */
    public double getThrottleWithDeadband(boolean squared, double deadbandThreshold)
    {
        return adjustValueWithDeadband(getThrottle(), squared, deadbandThreshold);
    }   //getThrottleWithDeadband

    /**
     * This method returns the value of the analog Throttle.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted value of the analog Throttle.
     */
    public double getThrottleWithDeadband(boolean squared)
    {
        return getThrottleWithDeadband(squared, deadbandThreshold);
    }   //getThrottleWithDeadband

    /**
     * This method returns the value of the analog magnitude.
     *
     * @param squared           specifies true to apply a squared curve to the output value, false otherwise.
     * @param deadbandThreshold specifies the deadband value to apply on this stick.
     * @return adjusted value of the analog magnitude.
     */
    public double getMagnitudeWithDeadband(boolean squared, double deadbandThreshold)
    {
        return adjustValueWithDeadband(getMagnitude(), squared, deadbandThreshold);
    }   //getMagnitudeWithDeadband

    /**
     * This method returns the value of the analog stick magnitude.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted value of the analog stick magnitude.
     */
    public double getMagnitudeWithDeadband(boolean squared)
    {
        return getMagnitudeWithDeadband(squared, deadbandThreshold);
    }   //getMagnitudeWithDeadband

    /**
     * This method returns the value of the analog stick direction in radians.
     *
     * @param squared           specifies true to apply a squared curve to the output value, false otherwise.
     * @param deadbandThreshold specifies the deadband value to apply on this stick.
     * @return adjusted value of the analog stick direction in radians.
     */
    public double getDirectionRadiansWithDeadband(boolean squared, double deadbandThreshold)
    {
        return adjustValueWithDeadband(getDirectionRadians(), squared, deadbandThreshold);
    }   //getDirectionRadiansWithDeadband

    /**
     * This method returns the value of the analog stick direction in radians.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted value of the analog stick direction in radians.
     */
    public double getDirectionRadiansWithDeadband(boolean squared)
    {
        return getDirectionRadiansWithDeadband(squared, deadbandThreshold);
    }   //getDirectionRadiansWithDeadband

    /**
     * This method returns the value of the analog stick direction in degrees.
     *
     * @param squared           specifies true to apply a squared curve to the output value, false otherwise.
     * @param deadbandThreshold specifies the deadband value to apply on this stick.
     * @return adjusted value of the analog stick direction in degrees.
     */
    public double getDirectionDegreesWithDeadband(boolean squared, double deadbandThreshold)
    {
        return adjustValueWithDeadband(getDirectionDegrees(), squared, deadbandThreshold);
    }   //getDirectionDegreesWithDeadband

    /**
     * This method returns the value of the analog stick direction in degrees.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted value of the analog stick direction in degrees.
     */
    public double getDirectionDegreesWithDeadband(boolean squared)
    {
        return getDirectionDegreesWithDeadband(squared, deadbandThreshold);
    }   //getDirectionDegreesWithDeadband

    /**
     * This method returns the state of the button.
     *
     * @param buttonID specifies the button to check its state.
     * @return true if the button is pressed, false if released.
     */
    public boolean isButtonPressed(int buttonID)
    {
        return (DriverStation.getStickButtons(port) & buttonID) != 0;
    }   //isButtonPressed

    /**
     * This method runs periodically and checks for changes in the button states. If any button changed state,
     * the button handler is called if one exists.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the current robot run mode.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void buttonEventTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            double currTime = TrcTimer.getCurrentTime();

            if (currTime >= nextPeriod)
            {
                nextPeriod = currTime + samplingPeriod;

                int currButtons = DriverStation.getStickButtons(port);
                if (buttonHandler != null && runMode != TrcRobot.RunMode.DISABLED_MODE)
                {
                    int changedButtons = prevButtons ^ currButtons;

                    while (changedButtons != 0)
                    {
                        //
                        // buttonMask contains the least significant set bit.
                        //
                        int buttonMask = changedButtons & ~(changedButtons ^ -changedButtons);
                        boolean pressed = (currButtons & buttonMask) != 0;
                        int buttonValue = TrcUtil.leastSignificantSetBitPosition(buttonMask);

                        tracer.traceDebug(instanceName, "button=" + buttonValue + ", pressed=" + pressed);
                        if (pressed)
                        {
                            //
                            // Button is pressed.
                            //
                            buttonHandler.buttonEvent(buttonValue, true);
                        }
                        else
                        {
                            //
                            // Button is released.
                            //
                            buttonHandler.buttonEvent(buttonValue, false);
                        }
                        //
                        // Clear the least significant set bit.
                        //
                        changedButtons &= ~buttonMask;
                    }
                }
                prevButtons = currButtons;
            }
        }
    }   //buttonEventTask

    /**
     * This method applies deadband to the value and squared the output if necessary.
     *
     * @param value             specifies the value to be processed.
     * @param squared           specifies true to apply a squared curve to the output value, false otherwise.
     * @param deadbandThreshold specifies the deadband value to apply to the value.
     * @return adjusted value.
     */
    private double adjustValueWithDeadband(double value, boolean squared, double deadbandThreshold)
    {
        value = (Math.abs(value) >= deadbandThreshold) ? value : 0.0;

        if (squared)
        {
            value = Math.signum(value) * value * value;
        }

        return value;
    }   //adjustValueWithDeadband

}   //class FrcJoystick
