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

package frclib;

import edu.wpi.first.wpilibj.Servo;
import trclib.TrcDbgTrace;
import trclib.TrcServo;

/**
 * This class implements an FRC specific servo motor that connects to a specified PWM channel on the RoboRIO.
 * This class extends TrcServo and is implementing the abstract methods required by TrcServo.
 */

 public class FrcServo extends TrcServo
{
    private Servo servo;
    private boolean inverted = false;
    private double prevLogicalPos = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pwmChannel specifies the PWM channel the servo is connected to.
     */
    public FrcServo(final String instanceName, int pwmChannel)
    {
        super(instanceName);

        this.servo = new Servo(pwmChannel);
        prevLogicalPos = servo.get();
    }   //FrcServo

    //
    // Implements TrcServo abstract methods.
    //

    /**
     * This method inverts the servo motor direction.
     *
     * @param inverted specifies true to invert the servo direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        final String funcName = "setInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.inverted = inverted;
    }   //setInverted

    /**
     * This method checks if the servo motor direction is inverted.
     *
     * @return true if the servo direction is inverted, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        final String funcName = "isInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", inverted);
        }

        return inverted;
    }   //isInverted

    /**
     * This method sets the servo logical position, mapping [0,1] => [min,max].
     * <p>
     * On a 180-degree servo, 0.0 is at 0-degree and 1.0 is at 180-degree. For a 90-degree servo, 0->0deg, 1->90deg.
     * If servo direction is inverted, then 0.0 is at 180-degree and 1.0 is at 0-degree. On a continuous servo,
     * 0.0 is rotating full speed in reverse, 0.5 is to stop the motor and 1.0 is rotating the motor full speed
     * forward. Again, motor direction can be inverted if setInverted is called.
     *
     * @param position specifies the logical servo position.
     */
    @Override
    public void setLogicalPosition(double position)
    {
        final String funcName = "setLogicalPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "position=%f", position);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.prevLogicalPos = inverted? 1.0 - position: position;
        servo.set(prevLogicalPos);
    }   //setLogicalPosition

    /**
     * This method returns the logical position value set by the last setPosition call. Note that servo motors do not
     * provide real time position feedback. So getPosition doesn't actually return the current position.
     *
     * @return servo position value set by the last setPosition call.
     */
    @Override
    public double getLogicalPosition()
    {
        final String funcName = "getLogicalPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", prevLogicalPos);
        }

        return prevLogicalPos;
    }   //getLogicalPosition

}   //class FrcServo
