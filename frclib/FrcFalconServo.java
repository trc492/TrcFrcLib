/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.ctre.phoenix.motorcontrol.ControlMode;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcServo;
import TrcCommonLib.trclib.TrcUtil;

public class FrcFalconServo extends TrcServo
{
    private FrcCANFalcon falcon;
    private double degreesPerTick;
    private double lastSetPos = 0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName    specifies the instance name of the servo.
     * @param motor           the physical motor controller object.
     * @param pidCoefficients the pid coefficients used for motion magic. Don't forget kF!
     * @param degreesPerTick  degrees per native sensor unit measured by the talon.
     * @param maxSpeed        desired max speed of the motor, in degrees per second.
     * @param maxAccel        desired max acceleration of the motor, in degrees per second per second.
     */
    public FrcFalconServo(String instanceName, FrcCANFalcon falcon, TrcPidController.PidCoefficients pidCoefficients,
        double degreesPerTick, double maxSpeed, double maxAccel)
    {
        super(instanceName);
        this.falcon = falcon;
        this.degreesPerTick = degreesPerTick;

        falcon.motor.config_kP(0, pidCoefficients.kP);
        falcon.motor.config_kI(0, pidCoefficients.kI);
        falcon.motor.config_kD(0, pidCoefficients.kD);
        falcon.motor.config_kF(0, pidCoefficients.kF);
        falcon.motor.config_IntegralZone(0, TrcUtil.round(pidCoefficients.iZone));
        falcon.motor.configMotionCruiseVelocity(TrcUtil.round((maxSpeed / degreesPerTick) / 10));
        falcon.motor.configMotionAcceleration(TrcUtil.round((maxAccel / degreesPerTick) / 10));
    }

    @Override
    public void setInverted(boolean inverted)
    {
        falcon.setInverted(inverted);
    }

    @Override
    public boolean isInverted()
    {
        return falcon.isInverted();
    }

    @Override
    public void setPosition(double position)
    {
        lastSetPos = position;
        double angle = position * 360.0;
        int ticks = TrcUtil.round(angle / degreesPerTick);
        falcon.motor.set(ControlMode.MotionMagic, ticks);
    }

    /**
     * Get the physical position of the motor, in degrees.
     *
     * @return Position in degrees.
     */
    @Override
    public double getEncoderPosition()
    {
        return falcon.getPosition() * degreesPerTick;
    }

    /**
     * Get the last set logical position of the motor. [0,1] => [0,360].
     * The position returned may not necessarily be in the range [0,1].
     *
     * @return The last set logical position.
     */
    @Override
    public double getPosition()
    {
        return lastSetPos;
    }

}   //class FrcFalconServo
