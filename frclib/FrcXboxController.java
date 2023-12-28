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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;

public class FrcXboxController extends XboxController
{
    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int BACK = 7;
    public static final int START = 8;
    public static final int LEFT_STICK_BUTTON = 9;
    public static final int RIGHT_STICK_BUTTON = 10;

    private static final double DEF_DEADBAND_THRESHOLD = 0.15;
    private static final double DEF_SAMPLING_PERIOD = 0.02;     //Sampling at 50Hz.
    private double samplingPeriod = DEF_SAMPLING_PERIOD;
    private double nextPeriod = 0.0;
    private double deadbandThreshold = DEF_DEADBAND_THRESHOLD;

    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final int port;
    private final TrcTaskMgr.TaskObject buttonEventTaskObj;
    private int prevButtons;
    private FrcButtonHandler buttonHandler = null;
    private int leftYSign = 1;
    private int rightYSign = 1;

    /**
     * Construct an instance of a xbox controller. The index is the USB port on the drivers
     * station.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public FrcXboxController(String instanceName, int port)
    {
        super(port);

        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.port = port;
        buttonEventTaskObj = TrcTaskMgr.createTask(instanceName + ".buttonEvent", this::buttonEventTask);
        prevButtons = DriverStation.getStickButtons(port);
    }   //FrcXboxController

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
    public void setLeftYInverted(boolean inverted)
    {
        leftYSign = inverted ? -1 : 1;
    }   //setYInverted

    /**
     * This method inverts the y-axis of the analog sticks.
     *
     * @param inverted specifies true if inverting the y-axis, false otherwise.
     */
    public void setRightYInverted(boolean inverted)
    {
        rightYSign = inverted ? -1 : 1;
    }   //setYInverted

    /**
     * This method returns the X value of the left analog stick.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted X value of the left analog stick.
     */
    public double getLeftXWithDeadband(boolean squared)
    {
        return adjustValueWithDeadband(getLeftX(), squared, deadbandThreshold);
    }

    /**
     * This method returns the X value of the right analog stick.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted X value of the right analog stick.
     */
    public double getRightXWithDeadband(boolean squared)
    {
        return adjustValueWithDeadband(getRightX(), squared, deadbandThreshold);
    }

    /**
     * This method overrides the method from the super class so that it will respect our inverted call.
     */
    @Override
    public double getLeftY()
    {
        return leftYSign * super.getLeftY();
    }   //getLeftY

    /**
     * This method overrides the method from the super class so that it will respect our inverted call.
     */
    @Override
    public double getRightY()
    {
        return rightYSign * super.getRightY();
    }   //getRightY

    /**
     * This method returns the Y value of the left analog stick.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted Y value of the left analog stick.
     */
    public double getLeftYWithDeadband(boolean squared)
    {
        return adjustValueWithDeadband(getLeftY(), squared, deadbandThreshold);
    }

    /**
     * This method returns the Y value of the right analog stick.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted Y value of the right analog stick.
     */
    public double getRightYWithDeadband(boolean squared)
    {
        return adjustValueWithDeadband(getRightY(), squared, deadbandThreshold);
    }

    /**
     * This method returns the value of the left analog trigger.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted value of the left analog trigger.
     */
    public double getLeftTriggerWithDeadband(boolean squared)
    {
        return adjustValueWithDeadband(getLeftTriggerAxis(), squared, deadbandThreshold);
    }

    /**
     * This method returns the value of the right analog trigger.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted value of the right analog trigger.
     */
    public double getRightTriggerWithDeadband(boolean squared)
    {
        return adjustValueWithDeadband(getRightTriggerAxis(), squared, deadbandThreshold);
    }

    /**
     * This method returns the combined value of the left and right analog triggers to give you a value range of
     * -1 to 1.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted value of the combined analog triggers.
     */
    public double getTriggerWithDeadband(boolean squared)
    {
        return adjustValueWithDeadband(getRightTriggerAxis() - getLeftTriggerAxis(), squared, deadbandThreshold);
    }

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
                        int buttonNum = TrcUtil.leastSignificantSetBitPosition(buttonMask) + 1;

                        tracer.traceDebug(instanceName, "button=" + buttonNum + ", pressed=" + pressed);
                        if (pressed)
                        {
                            //
                            // Button is pressed.
                            //
                            buttonHandler.buttonEvent(buttonNum, true);
                        }
                        else
                        {
                            //
                            // Button is released.
                            //
                            buttonHandler.buttonEvent(buttonNum, false);
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

}   //class FrcXboxController
