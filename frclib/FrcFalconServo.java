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
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcServo;
import TrcCommonLib.trclib.TrcUtil;

public class FrcFalconServo extends TrcServo
{
    private final FrcCANFalcon falcon;
    private final double degreesPerTick;
    private final double zeroPos;
    private TalonFXControlMode controlMode = TalonFXControlMode.Position;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName    specifies the instance name of the servo.
     * @param motor           the physical motor controller object.
     * @param pidCoefficients the pid coefficients used for motion magic. Don't forget kF!
     * @param degreesPerTick  degrees per native sensor unit measured by the talon.
     * @param zeroPos         encoder vaue when motor is at zero angle.
     * @param maxSpeed        desired max speed of the motor, in degrees per second.
     * @param maxAccel        desired max acceleration of the motor, in degrees per second per second.
     */
    public FrcFalconServo(String instanceName, FrcCANFalcon falcon, TrcPidController.PidCoefficients pidCoefficients,
        double degreesPerTick, double zeroPos, double maxSpeed, double maxAccel)
    {
        super(instanceName);
        this.falcon = falcon;
        this.degreesPerTick = degreesPerTick;
        this.zeroPos = zeroPos;

        // Set deadband to super small 0.001 (0.1 %). The default deadband is 0.04 (4 %).
        falcon.motor.configNeutralDeadband(0.001, 30);
        // Set relevant frame periods to be at least as fast as periodic rate.
        falcon.motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
        falcon.motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);
        // Set the peak and nominal outputs.
        falcon.motor.configNominalOutputForward(0, 30);
        falcon.motor.configNominalOutputReverse(0, 30);
        falcon.motor.configPeakOutputForward(1, 30);
        falcon.motor.configPeakOutputReverse(-1, 30);
        // Set Motion Magic gains in slot0 - see documentation.
        falcon.motor.selectProfileSlot(0, 0);
        falcon.motor.config_kP(0, pidCoefficients.kP, 30);
        falcon.motor.config_kI(0, pidCoefficients.kI, 30);
        falcon.motor.config_kD(0, pidCoefficients.kD, 30);
        falcon.motor.config_kF(0, pidCoefficients.kF, 30);
        falcon.motor.config_IntegralZone(0, TrcUtil.round(pidCoefficients.iZone), 30);
        // Set acceleration and vcruise velocity - see documentation.
        falcon.motor.configMotionCruiseVelocity(TrcUtil.round((maxSpeed / degreesPerTick) / 10), 30);
        falcon.motor.configMotionAcceleration(TrcUtil.round((maxAccel / degreesPerTick) / 10), 30);
    }   //FrcFalconServo

    /**
     * This method sets the motor control mode. Default control mode is Position mode.
     *
     * @param controlMode specifies the control mode to use in setting the motor position.
     */
    public void setControlMode(TalonFXControlMode controlMode)
    {
        this.controlMode = controlMode;
    }   //setControlMode

    /**
     * This method inverts the servo motor direction.
     *
     * @param inverted specifies the servo direction is inverted if true.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        falcon.setMotorInverted(inverted);
    }   //setInverted

    /**
     * This method checks if the servo motor direction is inverted.
     *
     * @return true if the servo direction is inverted, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        return falcon.isMotorInverted();
    }   //isInverted

    /**
     * This method sets the logical position of the servo motor.
     *
     * @param position specifies the logical position of the servo motor in the range of [0.0, 1.0].
     */
    @Override
    public void setLogicalPosition(double position)
    {
        double encPos = position * 360.0 / degreesPerTick + zeroPos;
        falcon.motor.set(controlMode, encPos);
    }   //setLogicalPosition

    /**
     * Get the current logical position of the motor. [0,1] => [0,360].
     *
     * @return current logical position.
     */
    @Override
    public double getLogicalPosition()
    {
        double physicalPos = ((falcon.getPosition() - zeroPos)*degreesPerTick) % 360.0;
        return physicalPos/360.0;
    }   //getLogicalPosition

    /**
     * This method stops a continuous servo. It doesn't do anything if the servo is not continuous.
     */
    public void stopContinuous()
    {
        throw new RuntimeException("FrcFalconServo does not support continuous mode.");
    }   //stopContinuous

}   //class FrcFalconServo
