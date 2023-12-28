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

import edu.wpi.first.wpilibj.Servo;
import TrcCommonLib.trclib.TrcServo;

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
    public FrcServo(String instanceName, int pwmChannel)
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
        return inverted;
    }   //isInverted

    /**
     * This method sets the logical position of the servo motor.
     *
     * @param position specifies the logical position of the servo motor in the range of [0.0, 1.0].
     */
    @Override
    public void setLogicalPosition(double position)
    {
        this.prevLogicalPos = inverted? 1.0 - position: position;
        servo.set(prevLogicalPos);
    }   //setLogicalPosition

    /**
     * This method returns the logical position value set by the last setLogicalPosition call. Note that servo motors
     * do not provide real time position feedback. Therefore, getLogicalPosition doesn't actually return the current
     * position.
     *
     * @return motor position value set by the last setLogicalPosition call in the range of [0.0, 1.0].
     */
    @Override
    public double getLogicalPosition()
    {
        return inverted? 1.0 - prevLogicalPos: prevLogicalPos;
    }   //getLogicalPosition

}   //class FrcServo
