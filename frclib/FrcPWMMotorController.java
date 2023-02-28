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
import TrcCommonLib.trclib.TrcPidController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class FrcPWMMotorController<T extends PWMMotorController> extends TrcMotor
{
    public final T motor;
    private final Encoder encoder;
    private final FrcDigitalInput revLimitSwitch, fwdLimitSwitch;
    private double batteryNominalVoltage = 0.0;
    private Double motorPower = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pwmMotor specifies the pwm motor object.
     * @param encoder specifies the encoder object, can be null if not provided.
     * @param revLimitSwitch specifies the reverse limit switch, can be null if not provided.
     * @param fwdLimitSwitch specifies the forward limit switch, can be null if not provided.
     */
    public FrcPWMMotorController(
        String instanceName, T pwmMotor, Encoder encoder,
        FrcDigitalInput revLimitSwitch, FrcDigitalInput fwdLimitSwitch)
    {
        super(instanceName);
        motor = pwmMotor;
        this.encoder = encoder;
        this.revLimitSwitch = revLimitSwitch;
        this.fwdLimitSwitch = fwdLimitSwitch;
    }   //FrcPWMMotorController

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pwmMotor specifies the pwm motor object.
     */
    public FrcPWMMotorController(String instanceName, T pwmMotor)
    {
        this(instanceName, pwmMotor, null, null, null);
    }   //FrcPWMMotorController

    //
    // Implements TrcMotorController interface.
    //

    /**
     * This method is used to check if the motor controller supports close loop control internally.
     *
     * @return true if motor controller supports close loop control, false otherwise.
     */
    public boolean supportCloseLoopControl()
    {
        return false;
    }   // supportCloseLoopControl

    /**
     * This method resets the motor controller configurations to factory default so that everything is at known state.
     */
    @Override
    public void resetFactoryDefault()
    {
        throw new UnsupportedOperationException("PWM motor controllers do not support this operation.");
    }   //resetFactoryDefault

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
     * This method sets this motor to follow another motor.
     *
     * @param otherMotor specifies the other motor to follow.
     */
    @Override
    public void followMotor(TrcMotor otherMotor)
    {
        otherMotor.addFollowingMotor(this);
    }   //followMotor

    /**
     * This method returns the bus voltage of the motor controller. PWM motor controllers do not really support this,
     * so we assume bus voltage is the same as battery voltage.
     *
     * @return bus voltage of the motor controller.
     */
    @Override
    public double getBusVoltage()
    {
        return RobotController.getBatteryVoltage();
    }   //getBusVoltage

    /**
     * This method enables voltage compensation so that it will maintain the motor output regardless of battery
     * voltage.
     *
     * @param batteryNominalVoltage specifies the nominal voltage of the battery.
     */
    @Override
    public void enableVoltageCompensation(double batteryNominalVoltage)
    {
        this.batteryNominalVoltage = batteryNominalVoltage;
    }   //enableVoltageCompensation

    /**
     * This method disables voltage compensation
     */
    @Override
    public void disableVoltageCompensation()
    {
        batteryNominalVoltage = 0.0;
    }   //disableVoltageCompensation

    /**
     * This method checks if voltage compensation is enabled.
     *
     * @return true if voltage compensation is enabled, false if disabled.
     */
    @Override
    public boolean isVoltageCompensationEnabled()
    {
        return batteryNominalVoltage != 0.0;
    }   //isVoltageCompensationEnabled

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would stop the motor very
     * abruptly by shorting the motor wires together using the generated back EMF to stop the motor. When not enabled,
     * (i.e. float/coast mode), the motor wires are just disconnected from the motor controller so the motor will
     * stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     */
    @Override
    public void setBrakeModeEnabled(boolean enabled)
    {
        throw new UnsupportedOperationException("PWM motor controllers do not support this operation.");
    }   //setBrakeModeEnabled

    /**
     * This method sets the PID coefficients of the motor's PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        throw new UnsupportedOperationException("PWM motor controllers do not support this operation.");
    }   //setPidCoefficients

    /**
     * This method returns the PID coefficients of the motor's PID controller.
     *
     * @return PID coefficients of the motor's PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getPidCoefficients()
    {
        throw new UnsupportedOperationException("PWM motor controllers do not support this operation.");
    }   //getPidCoefficients

    /**
     * This method inverts the active state of the reverse limit switch, typically reflecting whether the switch is
     * wired normally open or normally close.
     *
     * @param inverted specifies true to invert the limit switch, false otherwise.
     */
    @Override
    public void setRevLimitSwitchInverted(boolean inverted)
    {
        if (revLimitSwitch != null)
        {
            revLimitSwitch.setInverted(inverted);
        }
    }   //setRevLimitSwitchInverted

    /**
     * This method inverts the active state of the forward limit switch, typically reflecting whether the switch is
     * wired normally open or normally close.
     *
     * @param inverted specifies true to invert the limit switch, false otherwise.
     */
    @Override
    public void setFwdLimitSwitchInverted(boolean inverted)
    {
        if (fwdLimitSwitch != null)
        {
            fwdLimitSwitch.setInverted(inverted);
        }
    }   //setFwdLimitSwitchInverted

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    @Override
    public boolean isRevLimitSwitchActive()
    {
        return revLimitSwitch != null && revLimitSwitch.isActive();
    }   //isRevLimitSwitchClosed

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    @Override
    public boolean isFwdLimitSwitchActive()
    {
        return fwdLimitSwitch != null && fwdLimitSwitch.isActive();
    }   //isFwdLimitSwitchActive

    /**
     * This method inverts the spinning direction of the motor.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    @Override
    public void setMotorInverted(boolean inverted)
    {
        motor.setInverted(inverted);
    }   //setMotorInverted

    /**
     * This method checks if the motor direction is inverted.
     *
     * @return true if motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isMotorInverted()
    {
        return motor.getInverted();
    }   //isMotorInverted

    /**
     * This method stops the motor regardless of what control mode the motor is on.
     */
    public void stopMotor()
    {
        if (motorPower == null || motorPower != 0.0)
        {
            motor.stopMotor();
            motorPower = 0.0;
        }
    }   //stopMotor

    /**
     * This method sets the raw motor power.
     *
     * @param power specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    @Override
    public void setMotorPower(double power)
    {
        // Do this only if power is different from the previous set power.
        if (motorPower == null || motorPower != power)
        {
            motor.set(power);
            motorPower = power;
        }
    }   //setMotorPower

    /**
     * This method gets the current motor power.
     *
     * @return current motor power.
     */
    @Override
    public double getMotorPower()
    {
        // Motor power might have changed by closed loop controls, so get it directly from the motor.
        motorPower = motor.get();
        return motorPower;
    }   //getMotorPower

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

    /**
     * This method returns the state of the position sensor direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isPositionSensorInverted()
    {
        return encoder != null && encoder.getDirection();
    }   //isPositionSensorInverted

    /**
     * This method commands the motor to go to the given position using close loop control.
     *
     * @param position specifies the motor position in number of rotations.
     */
    @Override
    public void setMotorPosition(double position)
    {
        throw new UnsupportedOperationException("PWM motor controllers do not support this operation.");
    }   //setMotorPosition

    /**
     * This method returns the motor position by reading the position sensor.
     *
     * @return current motor position in raw sensor units.
     */
    @Override
    public double getMotorPosition()
    {
        return encoder != null? encoder.get(): 0.0;
    }   //getMotorPosition

    /**
     * This method resets the motor position sensor, typically an encoder.
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
     * This method commands the motor to spin at the given velocity using close loop control.
     *
     * @param velocity specifies the motor velocity in rotations per second.
     */
    @Override
    public void setMotorVelocity(double velocity)
    {
        throw new UnsupportedOperationException("PWM motor controllers do not support this operation.");
    }   //setMotorVelocity

    /**
     * This method returns the current motor velocity.
     *
     * @return current motor velocity in counts per second.
     */
    @Override
    public double getMotorVelocity()
    {
        return encoder != null? encoder.getRate()/encoder.getDistancePerPulse(): 0.0;
    }   //getMotorVelocity

    /**
     * This method commands the motor to spin at the given current value using close loop control.
     *
     * @param current specifies current in amperes.
     */
    @Override
    public void setMotorCurrent(double current)
    {
        throw new UnsupportedOperationException("PWM motor controllers do not support this operation.");
    }   //setMotorCurrent

    /**
     * This method returns the motor current.
     *
     * @return motor current in amperes.
     */
    @Override
    public double getMotorCurrent()
    {
        throw new UnsupportedOperationException("PWM motor controllers do not support this operation.");
    }   //getMotorCurrent

    /**
     * This method sets the close loop percentage output limits. By default the limits are set to the max at -1 to 1.
     * By setting a non-default limits, it effectively limits the output power of the close loop control.
     *
     * @param revLimit specifies the percentage output limit of the reverse direction.
     * @param fwdLimit specifies the percentage output limit of the forward direction.
     */
    @Override
    public void setCloseLoopOutputLimits(double revLimit, double fwdLimit)
    {
        throw new UnsupportedOperationException("PWM motor controller does not support this operation.");
    }   //setCloseLoopOutputLimits

   /**
     * This method sets the current limit of the motor.
     *
     * @param currentLimit specifies the current limit (holding current) in amperes when feature is activated.
     * @param triggerThresholdCurrent not used. SparkMax does not support this.
     * @param triggerThresholdTime not used. SparkMax does not support this.
     */
    @Override
    public void setCurrentLimit(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime)
    {
        throw new UnsupportedOperationException("PWM motor controller does not support this operation.");
    }   //setCurrentLimit

}   //class FrcPWMMotorController
