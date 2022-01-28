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
import TrcCommonLib.trclib.TrcUtil;

public class FrcXboxController extends XboxController
{
    private static final String moduleName = "FrcXboxController";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

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

    private final String instanceName;
    private final int port;
    private int prevButtons;
    private FrcButtonHandler buttonHandler = null;
    private int leftYSign = 1;
    private int rightYSign = 1;
    private TrcDbgTrace buttonEventTracer = null;

    /**
     * Construct an instance of a xbox controller. The index is the USB port on the drivers
     * station.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public FrcXboxController(String instanceName, int port)
    {
        super(port);

        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer ?
                TrcDbgTrace.getGlobalTracer() :
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.port = port;
        this.instanceName = instanceName;
        prevButtons = DriverStation.getStickButtons(port);

        TrcTaskMgr.TaskObject buttonEventTaskObj = TrcTaskMgr.createTask(
            instanceName + ".buttonEvent", this::buttonEventTask);
        buttonEventTaskObj.registerTask(TrcTaskMgr.TaskType.PREPERIODIC_TASK);
    }

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
     * This method enables/disables button event tracing by setting the event tracer.
     *
     * @param buttonEventTracer specifies the tracer to use when enabling button events tracing, null to disable
     *                          event tracing.
     */
    public void setButtonEventTracer(TrcDbgTrace buttonEventTracer)
    {
        final String funcName = "setButtonEventTracer";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "buttonTracing=%s",
                buttonEventTracer != null ? "enabled" : "disabled");
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.buttonEventTracer = buttonEventTracer;
    }   //setButtonEventTracer

    /**
     * This method sets the object that will handle button events. Any previous handler set with this method will
     * no longer receive events.
     *
     * @param buttonHandler specifies the object that will handle the button events. Set to null clear previously
     *                      set handler.
     */
    public void setButtonHandler(FrcButtonHandler buttonHandler)
    {
        final String funcName = "setButtonHandler";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "buttonHandler=%s", buttonHandler);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.buttonHandler = buttonHandler;
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
        final String funcName = "setSamplingPeriod";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "period=%.3f", period);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        samplingPeriod = period;
    }   //setSamplingPeriod

    /**
     * This method inverts the y-axis of the analog sticks.
     *
     * @param inverted specifies true if inverting the y-axis, false otherwise.
     */
    public void setLeftYInverted(boolean inverted)
    {
        final String funcName = "setLeftYInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", Boolean.toString(inverted));
        }

        leftYSign = inverted ? -1 : 1;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setYInverted

    /**
     * This method inverts the y-axis of the analog sticks.
     *
     * @param inverted specifies true if inverting the y-axis, false otherwise.
     */
    public void setRightYInverted(boolean inverted)
    {
        final String funcName = "setRightYInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", Boolean.toString(inverted));
        }

        rightYSign = inverted ? -1 : 1;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setYInverted

    /**
     * This method returns the X value of the left analog stick.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted X value of the left analog stick.
     */
    public double getLeftXWithDeadband(boolean squared)
    {
        final String funcName = "getLeftXWithDeadband";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "squared=%s,dbThreshold=%f",
                Boolean.toString(squared), deadbandThreshold);
        }

        double value = adjustValueWithDeadband(getLeftX(), squared, deadbandThreshold);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }

    /**
     * This method returns the X value of the right analog stick.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted X value of the right analog stick.
     */
    public double getRightXWithDeadband(boolean squared)
    {
        final String funcName = "getRightXWithDeadband";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "squared=%s,dbThreshold=%f",
                Boolean.toString(squared), deadbandThreshold);
        }

        double value = adjustValueWithDeadband(getRightX(), squared, deadbandThreshold);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }

    /**
     * This method returns the Y value of the left analog stick.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted Y value of the left analog stick.
     */
    public double getLeftYWithDeadband(boolean squared)
    {
        final String funcName = "getLeftYWithDeadband";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "squared=%s,dbThreshold=%f",
                Boolean.toString(squared), deadbandThreshold);
        }

        double value = leftYSign * adjustValueWithDeadband(getLeftY(), squared, deadbandThreshold);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }

    /**
     * This method returns the Y value of the right analog stick.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted Y value of the right analog stick.
     */
    public double getRightYWithDeadband(boolean squared)
    {
        final String funcName = "getRightYWithDeadband";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "squared=%s,dbThreshold=%f",
                Boolean.toString(squared), deadbandThreshold);
        }

        double value = rightYSign * adjustValueWithDeadband(getRightY(), squared, deadbandThreshold);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }

    /**
     * This method returns the value of the left analog trigger.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted value of the left analog trigger.
     */
    public double getLeftTriggerWithDeadband(boolean squared)
    {
        final String funcName = "getLeftTriggerWithDeadband";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "squared=%s,dbThreshold=%f",
                Boolean.toString(squared), deadbandThreshold);
        }

        double value = adjustValueWithDeadband(getLeftTriggerAxis(), squared, deadbandThreshold);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }

    /**
     * This method returns the value of the right analog trigger.
     *
     * @param squared specifies true to apply a squared curve to the output value, false otherwise.
     * @return adjusted value of the right analog trigger.
     */
    public double getRightTriggerWithDeadband(boolean squared)
    {
        final String funcName = "getRightTriggerWithDeadband";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "squared=%s,dbThreshold=%f",
                Boolean.toString(squared), deadbandThreshold);
        }

        double value = adjustValueWithDeadband(getRightTriggerAxis(), squared, deadbandThreshold);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }

    /**
     * This method runs periodically and checks for changes in the button states. If any button changed state,
     * the button handler is called if one exists.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode  specifies the current robot run mode.
     */
    private void buttonEventTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "buttonEventTask";
        double currTime = TrcUtil.getCurrentTime();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

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

                    if (buttonEventTracer != null)
                    {
                        buttonEventTracer
                            .traceInfo(funcName, "[%.3f] controller=%s, button=%d, pressed=%b", currTime, instanceName,
                                buttonNum, pressed);
                    }

                    if (pressed)
                    {
                        //
                        // Button is pressed.
                        //
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Button %x pressed", buttonNum);
                        }
                        buttonHandler.buttonEvent(buttonNum, true);
                    }
                    else
                    {
                        //
                        // Button is released.
                        //
                        if (debugEnabled)
                        {
                            dbgTrace.traceInfo(funcName, "Button %x released", buttonNum);
                        }
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

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
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
}
