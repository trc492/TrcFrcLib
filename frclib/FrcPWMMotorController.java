/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

import TrcCommonLib.trclib.TrcMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class FrcPWMMotorController<T extends PWMMotorController> extends TrcMotor
{
    public final T motor;
    private final Encoder encoder;
    private final FrcDigitalInput revLimitSw, fwdLimitSw;
    private double motorPower = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pwmMotor specifies the pwm motor object.
     * @param encoder specifies the encoder object, can be null if not provided.
     * @param revLimitSw specifies the reverse limit switch, can be null if not provided.
     * @param fwdLimitSw specifies the forward limit switch, can be null if not provided.
     */
    public FrcPWMMotorController(
        String instanceName, T pwmMotor, Encoder encoder, FrcDigitalInput revLimitSw, FrcDigitalInput fwdLimitSw)
    {
        super(instanceName);
        motor = pwmMotor;
        this.encoder = encoder;
        this.revLimitSw = revLimitSw;
        this.fwdLimitSw = fwdLimitSw;
        resetMotorPosition();
    }   //FrcPWMMotorController

    //
    // Implements TrcMotor abstract methods and overrides some of its methods supported in hardware.
    //

    /**
     * This method returns the state of the motor controller direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isMotorInverted()
    {
        return motor.getInverted();
    }   //isMotorInverted

    /**
     * This method inverts the motor direction.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    @Override
    public void setMotorInverted(boolean inverted)
    {
        motor.setInverted(inverted);
    }   //setMotorInverted

    /**
     * This method gets the last set power.
     *
     * @return the last setPower value.
     */
    @Override
    public double getMotorPower()
    {
        return motor.get();
    }   //getMotorPower

    /**
     * This method sets the raw motor power.
     *
     * @param value specifies the percentage power (range -1.0 to 1.0) to be set or sensor unit per second if
     *              velocity mode is enabled.
     */
    @Override
    public void setMotorPower(double value)
    {
        if (value != motorPower)
        {
            motor.set(value);
            // if (maxMotorVelocity != 0.0)
            // {
            //     // Velocity control mode.
            //     // Note: value is in the unit of sensor units per second but CTRE controllers want sensor units
            //     // per 100 msec so we need to divide value by 10.
            //     motor.set(ControlMode.Velocity, value/10.0);
            // }
            // else
            // {
            //     // PercentOutput control mode.
            //     motor.set(ControlMode.PercentOutput, value);
            // }
            motorPower = value;
        }
    }   //setMotorPower

    /**
     * This method returns the motor current.
     *
     * @return motor current.
     */
    @Override
    public double getMotorCurrent()
    {
        throw new UnsupportedOperationException("PWM motor controllers does not support getMotorCurrent.");
    }   //getMotorCurrent

    /**
     * This method stops the motor regardless of what control mode the motor is on.
     */
    public void stopMotor()
    {
        if (motorPower != 0.0)
        {
            motor.stopMotor();
            motorPower = 0.0;
        }
    }   //stopMotor

    /**
     * This method resets the motor position sensor, typically an encoder. This method emulates a reset for a
     * potentiometer.
     */
    @Override
    public void resetMotorPosition()
    {
        if (encoder != null)
        {
            encoder.reset();
        }
    }   //resetMotorPosition

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position in raw sensor units.
     */
    @Override
    public double getMotorPosition()
    {
        return encoder != null? encoder.get(): 0.0;
    }   //getMotorPosition

    /**
     * This method returns the motor velocity from the platform dependent motor hardware. If the hardware does
     * not support velocity info, it should throw an UnsupportedOperationException.
     *
     * @return current motor velocity in raw sensor units per sec.
     */
    @Override
    public double getMotorVelocity()
    {
        return encoder != null? encoder.getRate()/encoder.getDistancePerPulse(): 0.0;
    }   //getMotorVelocity

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    @Override
    public boolean isRevLimitSwitchActive()
    {
        return revLimitSw != null? revLimitSw.isActive(): false;
    }   //isRevLimitSwitchActive

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    @Override
    public boolean isFwdLimitSwitchActive()
    {
        return fwdLimitSw != null? fwdLimitSw.isActive(): false;
    }   //isFwdLimitSwitchActive

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would stop the motor very
     * abruptly by shorting the motor wires together using the generated back EMF to stop the motor. When brakMode
     * is false (i.e. float/coast mode), the motor wires are just disconnected from the motor controller so the motor
     * will stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     */
    @Override
    public void setBrakeModeEnabled(boolean enabled)
    {
        throw new UnsupportedOperationException("PWM motor controllers do not support brake mode.");
    }   //setBrakeModeEnabled

    /**
     * This method checks if the device is connected to the robot. PWM motor controllers have no feedback
     * communication. There is no simple way to check this. Therefore, we will just say it's always connected.
     *
     * @return true always.
     */
    @Override
    public boolean isConnected()
    {
        return true;
    } //isConnected

    /**
     * This method inverts the position sensor direction. This may be rare but there are scenarios where the motor
     * encoder may be mounted somewhere in the power train that it rotates opposite to the motor rotation. This will
     * cause the encoder reading to go down when the motor is receiving positive power. This method can correct this
     * situation.
     *
     * @param inverted specifies true to invert position sensor direction, false otherwise.
     */
    @Override
    public void setPositionSensorInverted(boolean inverted)
    {
        if (encoder != null)
        {
            encoder.setReverseDirection(inverted);
        }
    }   //setPositionSensorInverted

}   //class FrcPWMMotorController
