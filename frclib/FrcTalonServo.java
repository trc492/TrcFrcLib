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

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcServo;
import TrcCommonLib.trclib.TrcUtil;

public class FrcTalonServo extends TrcServo
{
    private FrcCANTalon talon;
    private double degreesPerTick;
    private TalonSRXControlMode controlMode = TalonSRXControlMode.Position;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName    specifies the instance name of the servo.
     * @param talon           the physical talon motor controller object.
     * @param pidCoefficients the pid coefficients used for motion magic. Don't forget kF!
     * @param degreesPerTick  degrees per native sensor unit measured by the talon.
     * @param maxSpeed        desired max speed of the motor, in degrees per second.
     * @param maxAccel        desired max acceleration of the motor, in degrees per second per second.
     */
    public FrcTalonServo(String instanceName, FrcCANTalon talon, TrcPidController.PidCoefficients pidCoefficients,
        double degreesPerTick, double maxSpeed, double maxAccel)
    {
        super(instanceName);
        this.talon = talon;
        this.degreesPerTick = degreesPerTick;

        // Set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %).
        talon.motor.configNeutralDeadband(0.001, 30);
        // Set relevant frame periods to be at least as fast as periodic rate.
        talon.motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
        talon.motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
        // Set the peak and nominal outputs.
        talon.motor.configNominalOutputForward(0, 30);
        talon.motor.configNominalOutputReverse(0, 30);
        talon.motor.configPeakOutputForward(1, 30);
        talon.motor.configPeakOutputReverse(-1, 30);
        // Set Motion Magic gains in slot0 - see documentation.
        talon.motor.selectProfileSlot(0, 0);
        talon.motor.config_kP(0, pidCoefficients.kP, 30);
        talon.motor.config_kI(0, pidCoefficients.kI, 30);
        talon.motor.config_kD(0, pidCoefficients.kD, 30);
        talon.motor.config_kF(0, pidCoefficients.kF, 30);
        talon.motor.config_IntegralZone(0, TrcUtil.round(pidCoefficients.iZone), 30);
        // Set acceleration and vcruise velocity - see documentation.
        talon.motor.configMotionCruiseVelocity(TrcUtil.round((maxSpeed / degreesPerTick) / 10), 30);
        talon.motor.configMotionAcceleration(TrcUtil.round((maxAccel / degreesPerTick) / 10), 30);
    }   //FrcTalonServo

    /**
     * This method inverts the servo motor direction.
     *
     * @param inverted specifies the servo direction is inverted if true.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        talon.setMotorInverted(inverted);
    }   //setInverted

    /**
     * This method checks if the servo motor direction is inverted.
     *
     * @return true if the servo direction is inverted, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        return talon.isMotorInverted();
    }   //isInverted

    /**
     * This method sets the logical position of the servo motor.
     *
     * @param position specifies the logical position of the servo motor in the range of [0.0, 1.0].
     */
    @Override
    public void setLogicalPosition(double position)
    {
        int ticks = TrcUtil.round(position*360.0 / degreesPerTick);
        talon.motor.set(controlMode, ticks);
    }   //setLogicalPosition

    /**
     * Get the current logical position of the motor. [0,1] => [0,360].
     *
     * @return current logical position.
     */
    @Override
    public double getLogicalPosition()
    {
        double physicalPos = (talon.getPosition()*degreesPerTick) % 360.0;
        return physicalPos/360.0;
    }   //getLogicalPosition

    /**
     * This method stops a continuous servo. It doesn't do anything if the servo is not continuous.
     */
    public void stopContinuous()
    {
        throw new RuntimeException("FrcTalonServo does not support continuous mode.");
    }   //stopContinuous

}   //class FrcTalonServo
