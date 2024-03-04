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

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.AppliedRotorPolarityValue;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcEncoder;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public abstract class FrcCANPhoenix6Controller<T extends CoreTalonFX> extends TrcMotor
{
    private static final int PIDSLOT_POSITION = 0;
    private static final int PIDSLOT_VELOCITY = 1;

    public final T motor;
    private TalonFXConfiguration talonFxConfigs = new TalonFXConfiguration();
    private Double batteryNominalVoltage = null;
    // TODO: To support Motion Profile
    // - Create a TrapezoidProfile with given maxVel and maxAccel
    // - Set up Coeffs: kP, kI, kD, kV, kS
    private boolean useMotionProfile = false;

    // The number of non-success error codes reported by the device after sending a command.
    private int errorCount = 0;
    private StatusCode lastStatus = null;

    private class EncoderInfo implements Sendable
    {
        @Override
        public void initSendable(SendableBuilder builder)
        {
            if (!talonFxConfigs.Feedback.FeedbackSensorSource.equals(FeedbackSensorSourceValue.RotorSensor))
            {
                throw new IllegalStateException("Only internal QuadEncoder supported for Shuffleboard!");
            }

            builder.setSmartDashboardType("Quadrature Encoder");
            builder.addDoubleProperty("Speed", FrcCANPhoenix6Controller.this::getVelocity, null);
            builder.addDoubleProperty("Distance", FrcCANPhoenix6Controller.this::getPosition, null);
            builder.addDoubleProperty("DistancePerCount", () -> 1, null);
        }   //initSendable
    }   //class EncoderInfo

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param coreTalonFx the base talon FX object.
     * @param lowerLimitSwitch specifies an external lower limit switch overriding the motor controller one.
     * @param upperLimitSwitch specifies an external upper limit switch overriding the motor controller one.
     * @param encoder specifies an external encoder overriding the motor controller one.
     */
    public FrcCANPhoenix6Controller(
        String instanceName, T coreTalonFx, TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch,
        TrcEncoder encoder)
    {
        super(instanceName, lowerLimitSwitch, upperLimitSwitch, encoder);
        motor = coreTalonFx;
        recordResponseCode("readConfigs", motor.getConfigurator().refresh(talonFxConfigs));
    }   //FrcCANPhoenix6Controller

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param coreTalonFx the base talon FX object.
     */
    public FrcCANPhoenix6Controller(String instanceName, T coreTalonFx)
    {
        this(instanceName, coreTalonFx, null, null, null);
    }   //FrcCANPhoenix6Controller

    /**
     * This method creates an EncoderInfo object and returns it.
     *
     * @return created GyroInfo object.
     */
    public Sendable getEncoderSendable()
    {
        EncoderInfo encoderInfo = new EncoderInfo();
        SendableRegistry.setName(encoderInfo, toString());
        return encoderInfo;
    }   //getEncoderSendable

    /**
     * This method returns the number of error responses seen from the motor after sending a command.
     *
     * @return The number of non-OK error code responses seen from the motor
     * after sending a command.
     */
    public int getErrorCount()
    {
        return errorCount;
    } //getErrorCount

    /**
     * The method returns the last error code. If there is none, null is returned.
     *
     * @return last error code.
     */
    public StatusCode getLastStatus()
    {
        return lastStatus;
    }   //getLastStatus

    /**
     * This method checks for error code returned by the motor controller executing the last command. If there was
     * an error, the error count is incremented.
     *
     * @param operation specifies the operation that failed.
     * @param statusCode specifies the status code returned by the motor controller.
     */
    protected StatusCode recordResponseCode(String operation, StatusCode statusCode)
    {
        lastStatus = statusCode;
        if (statusCode != null && !statusCode.equals(StatusCode.OK))
        {
            errorCount++;
            tracer.traceErr(instanceName, operation + " (StatusCode=" + statusCode + ")");
        }
        return statusCode;
    }   //recordResponseCode

    /**
     * This method sets the feedback sensor source.
     *
     * @param sensorSource specifies the feedback sensor source.
     * @param remoteSensorId specifies the CAN ID of the remote sensor (only applicable for remote sensor source),
     *        0 if not applicable.
     */
    public void setFeedbackDevice(FeedbackSensorSourceValue sensorSource, int remoteSensorId)
    {
        talonFxConfigs.Feedback.FeedbackSensorSource = sensorSource;
        talonFxConfigs.Feedback.FeedbackRemoteSensorID = remoteSensorId;
        recordResponseCode("setFeedbackDevice", motor.getConfigurator().apply(talonFxConfigs.Feedback));
    }   //setFeedbackDevice

    /**
     * This method sets the feedback sensor source.
     *
     * @param sensorSource specifies the feedback sensor source.
     */
    public void setFeedbackDevice(FeedbackSensorSourceValue sensorSource)
    {
        setFeedbackDevice(sensorSource, 0);
    }   //setFeedbackDevice

    //
    // Implements TrcMotorController interface.
    //

    // /**
    //  * This method is used to check if the motor controller supports close loop control natively.
    //  *
    //  * @return true if motor controller supports close loop control, false otherwise.
    //  */
    // @Override
    // public boolean supportCloseLoopControl()
    // {
    //     return true;
    // }   // supportCloseLoopControl

    // /**
    //  * This method checks if the device is connected to the robot.
    //  *
    //  * @return true if the device is connected, false otherwise.
    //  */
    // @Override
    // public boolean isConnected()
    // {
    //     // Hacky, but should work
    //     return motor.getSupplyVoltage().getValueAsDouble() > 0.0;
    // }   //isConnected

    /**
     * This method resets the motor controller configurations to factory default so that everything is at known state.
     */
    @Override
    public void resetFactoryDefault()
    {
        // Create a new TalonFX config which will contain all factory default configurations and apply it.
        talonFxConfigs = new TalonFXConfiguration();
        recordResponseCode("resetFactoryDefault", motor.getConfigurator().apply(talonFxConfigs));
    }   //resetFactoryDefault

    /**
     * This method returns the bus voltage of the motor controller.
     *
     * @return bus voltage of the motor controller.
     */
    @Override
    public double getBusVoltage()
    {
        return motor.getSupplyVoltage().getValueAsDouble();
    }   //getBusVoltage

    /**
     * This method sets the current limit of the motor.
     *
     * @param currentLimit specifies the current limit (holding current) in amperes when feature is activated.
     * @param triggerThresholdCurrent specifies threshold current in amperes to be exceeded before limiting occurs.
     *        If this value is less than currentLimit, then currentLimit is used as the threshold.
     * @param triggerThresholdTime specifies how long current must exceed threshold (seconds) before limiting occurs.
     */
    @Override
    public void setCurrentLimit(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime)
    {
        talonFxConfigs.CurrentLimits.SupplyCurrentLimit = currentLimit;
        talonFxConfigs.CurrentLimits.SupplyCurrentThreshold = triggerThresholdCurrent;
        talonFxConfigs.CurrentLimits.SupplyTimeThreshold = triggerThresholdTime;
        talonFxConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        recordResponseCode("setCurrentLimit", motor.getConfigurator().apply(talonFxConfigs.CurrentLimits));
    }   //setCurrentLimit

    /**
     * This method sets the stator current limit of the motor.
     *
     * @param currentLimit specifies the stator current limit in amperes.
     */
    @Override
    public void setStatorCurrentLimit(double currentLimit)
    {
        talonFxConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;
        talonFxConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        recordResponseCode("setStatorCurrentLimit", motor.getConfigurator().apply(talonFxConfigs.CurrentLimits));
    }   //setStatorCurrentLimit

    /**
     * This method sets the close loop ramp rate.
     *
     * @param rampTime specifies the ramp time in seconds from neutral to full speed.
     */
    @Override
    public void setCloseLoopRampRate(double rampTime)
    {
        talonFxConfigs.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = rampTime;
        recordResponseCode("setClosedLoopRampRate", motor.getConfigurator().apply(talonFxConfigs.ClosedLoopRamps));
    }   //setCloseLoopRampRate

    /**
     * This method sets the open loop ramp rate.
     *
     * @param rampTime specifies the ramp time in seconds from neutral to full speed.
     */
    @Override
    public void setOpenLoopRampRate(double rampTime)
    {
        talonFxConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;
        recordResponseCode("setOpenLoopRampRate", motor.getConfigurator().apply(talonFxConfigs.OpenLoopRamps));
    }   //setOpenLoopRampRate

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
        talonFxConfigs.MotorOutput.NeutralMode = enabled? NeutralModeValue.Brake: NeutralModeValue.Coast;
        recordResponseCode("setBrakeModeEnabled", motor.getConfigurator().apply(talonFxConfigs.MotorOutput));
    }   //setBrakeModeEnabled

    /**
     * This method enables the reverse limit switch and configures it to the specified type.
     *
     * @param normalClose specifies true as the normal close switch type, false as normal open.
     */
    @Override
    public void enableMotorRevLimitSwitch(boolean normalClose)
    {
        talonFxConfigs.HardwareLimitSwitch.ReverseLimitType =
            normalClose? ReverseLimitTypeValue.NormallyClosed: ReverseLimitTypeValue.NormallyOpen;
        talonFxConfigs.HardwareLimitSwitch.ReverseLimitEnable = true;
        recordResponseCode(
            "enableMotorRevLimitSwitch", motor.getConfigurator().apply(talonFxConfigs.HardwareLimitSwitch));
    }   //enableMotorRevLimitSwitch

    /**
     * This method enables the forward limit switch and configures it to the specified type.
     *
     * @param normalClose specifies true as the normal close switch type, false as normal open.
     */
    @Override
    public void enableMotorFwdLimitSwitch(boolean normalClose)
    {
        talonFxConfigs.HardwareLimitSwitch.ForwardLimitType =
            normalClose? ForwardLimitTypeValue.NormallyClosed: ForwardLimitTypeValue.NormallyOpen;
        talonFxConfigs.HardwareLimitSwitch.ForwardLimitEnable = true;
        recordResponseCode(
            "enableMotorFwdLimitSwitch", motor.getConfigurator().apply(talonFxConfigs.HardwareLimitSwitch));
    }   //enableMotorFwdLimitSwitch

    /**
     * This method disables the reverse limit switch.
     */
    @Override
    public void disableMotorRevLimitSwitch()
    {
        talonFxConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;
        recordResponseCode(
            "enableMotorRevLimitSwitch", motor.getConfigurator().apply(talonFxConfigs.HardwareLimitSwitch));
    }   //disableMotorRevLimitSwitch

    /**
     * This method disables the forward limit switch.
     */
    @Override
    public void disableMotorFwdLimitSwitch()
    {
        talonFxConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
        recordResponseCode(
            "enableMotorFwdLimitSwitch", motor.getConfigurator().apply(talonFxConfigs.HardwareLimitSwitch));
    }   //disableMotorFwdLimitSwitch

    /**
     * This method checks if the reverse limit switch is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    @Override
    public boolean isMotorRevLimitSwitchEnabled()
    {
        recordResponseCode(
            "isMotorRevLimitSwitchEnabled", motor.getConfigurator().refresh(talonFxConfigs.HardwareLimitSwitch));
        return talonFxConfigs.HardwareLimitSwitch.ReverseLimitEnable;
    }   //isMotorRevLimitSwitchEnabled

    /**
     * This method checks if the forward limit switch is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    @Override
    public boolean isMotorFwdLimitSwitchEnabled()
    {
        recordResponseCode(
            "isMotorFwdLimitSwitchEnabled", motor.getConfigurator().refresh(talonFxConfigs.HardwareLimitSwitch));
        return talonFxConfigs.HardwareLimitSwitch.ForwardLimitEnable;
    }   //isMotorFwdLimitSwitchEnabled

    /**
     * This method inverts the active state of the reverse limit switch, typically reflecting whether the switch is
     * wired normally open or normally close.
     *
     * @param inverted specifies true to invert the limit switch to normal close, false to normal open.
     */
    @Override
    public void setMotorRevLimitSwitchInverted(boolean inverted)
    {
        talonFxConfigs.HardwareLimitSwitch.ReverseLimitType =
            inverted? ReverseLimitTypeValue.NormallyClosed: ReverseLimitTypeValue.NormallyOpen;
        recordResponseCode(
            "setMotorRevLimitSwitchInverted", motor.getConfigurator().apply(talonFxConfigs.HardwareLimitSwitch));
    }   //setMotorRevLimitSwitchInverted

    /**
     * This method inverts the active state of the forward limit switch, typically reflecting whether the switch is
     * wired normally open or normally close.
     *
     * @param inverted specifies true to invert the limit switch to normal close, false to normal open.
     */
    @Override
    public void setMotorFwdLimitSwitchInverted(boolean inverted)
    {
        talonFxConfigs.HardwareLimitSwitch.ForwardLimitType =
            inverted? ForwardLimitTypeValue.NormallyClosed: ForwardLimitTypeValue.NormallyOpen;
        recordResponseCode(
            "setMotorFwdLimitSwitchInverted", motor.getConfigurator().apply(talonFxConfigs.HardwareLimitSwitch));
    }   //setMotorFwdLimitSwitchInverted

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    @Override
    public boolean isMotorRevLimitSwitchActive()
    {
        recordResponseCode(
            "isMotorRevLimitSwitchActive", motor.getConfigurator().refresh(talonFxConfigs.HardwareLimitSwitch));
        return (talonFxConfigs.HardwareLimitSwitch.ReverseLimitType.equals(ReverseLimitTypeValue.NormallyClosed)) ^
               (motor.getReverseLimit().getValue().equals(ReverseLimitValue.ClosedToGround));
    }   //isMotorRevLimitSwitchActive

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    @Override
    public boolean isMotorFwdLimitSwitchActive()
    {
        recordResponseCode(
            "isMotorFwdLimitSwitchActive", motor.getConfigurator().refresh(talonFxConfigs.HardwareLimitSwitch));
        return (talonFxConfigs.HardwareLimitSwitch.ForwardLimitType.equals(ForwardLimitTypeValue.NormallyClosed)) ^
               (motor.getForwardLimit().getValue().equals(ForwardLimitValue.ClosedToGround));
    }   //isMotorFwdLimitSwitchActive

    /**
     * This method sets the soft position limit for the reverse direction.
     *
     * @param limit specifies the limit in sensor units, null to disable.
     */
    @Override
    public void setMotorRevSoftPositionLimit(Double limit)
    {
        if (limit != null)
        {
            talonFxConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = limit;
            talonFxConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        }
        else
        {
            talonFxConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        }
        recordResponseCode(
            "setMotorRevSoftPositionLimit", motor.getConfigurator().apply(talonFxConfigs.SoftwareLimitSwitch));
    }   //setMotorRevSoftPositionLimit

    /**
     * This method sets the soft position limit for the forward direction.
     *
     * @param limit specifies the limit in sensor units, null to disable.
     */
    @Override
    public void setMotorFwdSoftPositionLimit(Double limit)
    {
        if (limit != null)
        {
            talonFxConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = limit;
            talonFxConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        }
        else
        {
            talonFxConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        }
        recordResponseCode(
            "setMotorFwdSoftPositionLimit", motor.getConfigurator().apply(talonFxConfigs.SoftwareLimitSwitch));
    }   //setMotorFwdSoftPositionLimit

    /**
     * This method inverts the position sensor direction. This may be rare but there are scenarios where the motor
     * encoder may be mounted somewhere in the power train that it rotates opposite to the motor rotation. This will
     * cause the encoder reading to go down when the motor is receiving positive power. This method can correct this
     * situation.
     *
     * @param inverted specifies true to invert position sensor direction, false otherwise.
     */
    @Override
    public void setMotorPositionSensorInverted(boolean inverted)
    {
        throw new UnsupportedOperationException("Controller does not support inverting internal position sensor.");
    }   //setMotorPositionSensorInverted

    /**
     * This method returns the state of the position sensor direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isMotorPositionSensorInverted()
    {
        throw new UnsupportedOperationException("Controller does not support inverting internal position sensor.");
    }   //isMotorPositionSensorInverted

    /**
     * This method resets the motor position sensor, typically an encoder.
     */
    @Override
    public void resetMotorPosition()
    {
        if (talonFxConfigs.Feedback.FeedbackSensorSource.equals(FeedbackSensorSourceValue.RotorSensor))
        {
            recordResponseCode("resetMotorPosition", motor.setPosition(0.0));
        }
    }   //resetMotorPosition

    /**
     * This method inverts the spinning direction of the motor.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    @Override
    public void setMotorInverted(boolean inverted)
    {
        talonFxConfigs.MotorOutput.Inverted =
            inverted? InvertedValue.Clockwise_Positive: InvertedValue.CounterClockwise_Positive;
        recordResponseCode(
            "setMotorInverted", motor.getConfigurator().apply(talonFxConfigs.MotorOutput));
    }   //setMotorInverted

    /**
     * This method checks if the motor direction is inverted.
     *
     * @return true if motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isMotorInverted()
    {
        return motor.getAppliedRotorPolarity().getValue().equals(AppliedRotorPolarityValue.PositiveIsClockwise);
    }   //isMotorInverted

    /**
     * This method sets the motor power.
     *
     * @param power specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    @Override
    public void setMotorPower(double power)
    {
        if (batteryNominalVoltage != null)
        {
            recordResponseCode(
                "setMotorPowerWithVolt", motor.setControl(new VoltageOut(power * batteryNominalVoltage)));
        }
        else
        {
            recordResponseCode("setMotorPower", motor.setControl(new DutyCycleOut(power)));
        }
    }   //setMotorPower

    /**
     * This method gets the current motor power.
     *
     * @return current percentage motor power.
     */
    @Override
    public double getMotorPower()
    {
        return motor.getDutyCycle().getValueAsDouble();
    }   //getMotorPower

    /**
     * This method commands the motor to spin at the given velocity using close loop control.
     *
     * @param velocity specifies the motor velocity in rotations per second.
     * @param acceleration specifies the max motor acceleration rotations per second square, can be 0 if not provided.
     * @param feedForward specifies feedforward in volts if voltage comp is ON, otherwise fractional unit between
     *        -1 and 1.
     */
    @Override
    public void setMotorVelocity(double velocity, double acceleration, double feedForward)
    {
        if (useMotionProfile)
        {
            if (batteryNominalVoltage != null)
            {
                recordResponseCode(
                    "setMotorVelocityWithVoltageAndMotionMagic", motor.setControl(
                        new MotionMagicVelocityVoltage(velocity).withAcceleration(acceleration)
                            .withFeedForward(feedForward).withSlot(PIDSLOT_VELOCITY)));
            }
            else
            {
                recordResponseCode(
                    "setMotorVelocityWithDutyCycleAndMotionMagic", motor.setControl(
                    new MotionMagicVelocityDutyCycle(velocity).withAcceleration(acceleration)
                        .withFeedForward(feedForward).withSlot(PIDSLOT_VELOCITY)));
            }
        }
        else
        {
            if (batteryNominalVoltage != null)
            {
                recordResponseCode(
                    "setMotorVelocityWithVoltage", motor.setControl(
                        new VelocityVoltage(velocity).withAcceleration(acceleration)
                            .withFeedForward(feedForward).withSlot(PIDSLOT_VELOCITY)));
            }
            else
            {
                recordResponseCode(
                    "setMotorVelocityWithDutyCycle", motor.setControl(
                    new VelocityDutyCycle(velocity).withAcceleration(acceleration)
                        .withFeedForward(feedForward).withSlot(PIDSLOT_VELOCITY)));
            }
        }
    }   //setMotorVelocity

    /**
     * This method returns the current motor velocity.
     *
     * @return current motor velocity in rotations per sec.
     */
    @Override
    public double getMotorVelocity()
    {
        return motor.getVelocity().getValueAsDouble();
    }   //getMotorVelocity

    /**
     * This method commands the motor to go to the given position using close loop control and optionally limits the
     * power of the motor movement.
     *
     * @param position specifies the position in rotations.
     * @param powerLimit specifies the maximum power output limits, can be null if not provided. If not provided, the
     *        previous set limit is applied.
     * @param velocity specifies the max motor veloicty rotations per second, can be 0 if not provided.
     * @param feedForward specifies feedforward in volts if voltage comp is ON, otherwise fractional unit between
     *        -1 and 1.
     */
    @Override
    public void setMotorPosition(double position, Double powerLimit, double velocity, double feedForward)
    {
        if (powerLimit != null)
        {
            // Set power limits.
            powerLimit = Math.abs(powerLimit);
            talonFxConfigs.MotorOutput.PeakForwardDutyCycle = powerLimit;
            talonFxConfigs.MotorOutput.PeakReverseDutyCycle = -powerLimit;
            recordResponseCode("setMotorPositionPowerLimit", motor.getConfigurator().apply(talonFxConfigs.MotorOutput));
        }

        if (useMotionProfile)
        {
            if (batteryNominalVoltage != null)
            {
                recordResponseCode(
                    "setMotorPositionWithVoltageAndMotionMagic", motor.setControl(
                        new MotionMagicVoltage(position).withFeedForward(feedForward).withSlot(PIDSLOT_POSITION)));
            }
            else
            {
                recordResponseCode(
                    "setMotorPositionWithDutyCycleAndMotionMagic", motor.setControl(
                        new MotionMagicDutyCycle(position).withFeedForward(feedForward).withSlot(PIDSLOT_POSITION)));
            }
        }
        else
        {
            if (batteryNominalVoltage != null)
            {
                recordResponseCode(
                    "setMotorPositionWithVoltage", motor.setControl(
                        new PositionVoltage(position).withVelocity(velocity)
                            .withFeedForward(feedForward).withSlot(PIDSLOT_POSITION)));
            }
            else
            {
                recordResponseCode(
                    "setMotorPositionWithDutyCycle", motor.setControl(
                        new PositionDutyCycle(position).withVelocity(velocity)
                            .withFeedForward(feedForward).withSlot(PIDSLOT_POSITION)));
            }
        }
    }   //setMotorPosition

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position in rotations.
     */
    @Override
    public double getMotorPosition()
    {
        return motor.getPosition().getValueAsDouble();
    }   //getMotorPosition

    /**
     * This method commands the motor to spin at the given current value using close loop control.
     *
     * @param current specifies current in amperes.
     */
    @Override
    public void setMotorCurrent(double current)
    {
        // This support requires Phoenix Pro.
        if (motor.getIsProLicensed().getValue())
        {
            recordResponseCode("setMotorCurrent", motor.setControl(new TorqueCurrentFOC(current)));
        }
        else
        {
            throw new UnsupportedOperationException("This operation requires Phoenix Pro.");
        }
    }   //setMotorCurrent

    /**
     * This method returns the motor current.
     *
     * @return motor current in amperes.
     */
    @Override
    public double getMotorCurrent()
    {
        return motor.getTorqueCurrent().getValueAsDouble();
    }   //getMotorCurrent

    /**
     * This method sets the PID coefficients of the the specified slot.
     *
     * @param slotIdx specifies the slot index.
     * @param pidCoeffs specifies the PID coefficients to set.
     */
    private void setPidCoefficients(int slotIdx, TrcPidController.PidCoefficients pidCoeffs)
    {
        switch (slotIdx)
        {
            case 0:
                talonFxConfigs.Slot0.kP = pidCoeffs.kP;
                talonFxConfigs.Slot0.kI = pidCoeffs.kI;
                talonFxConfigs.Slot0.kD = pidCoeffs.kD;
                talonFxConfigs.Slot0.kV = pidCoeffs.kF;
                recordResponseCode(
                    "setPidCoefficientsSlot0", motor.getConfigurator().apply(talonFxConfigs.Slot0));
                break;

            case 1:
                talonFxConfigs.Slot1.kP = pidCoeffs.kP;
                talonFxConfigs.Slot1.kI = pidCoeffs.kI;
                talonFxConfigs.Slot1.kD = pidCoeffs.kD;
                talonFxConfigs.Slot1.kV = pidCoeffs.kF;
                recordResponseCode(
                    "setPidCoefficientsSlot1", motor.getConfigurator().apply(talonFxConfigs.Slot1));
                break;

            case 2:
                talonFxConfigs.Slot2.kP = pidCoeffs.kP;
                talonFxConfigs.Slot2.kI = pidCoeffs.kI;
                talonFxConfigs.Slot2.kD = pidCoeffs.kD;
                talonFxConfigs.Slot2.kV = pidCoeffs.kF;
                recordResponseCode(
                    "setPidCoefficientsSlot2", motor.getConfigurator().apply(talonFxConfigs.Slot2));
                break;

            default:
                break;
        }
    }   //setPidCoefficients

    /**
     * This method returns the PID coefficients of the specified slot.
     *
     * @param slotIdx specifies the slot index.
     * @return PID coefficients of the specified slot.
     */
    private TrcPidController.PidCoefficients getPidCoefficients(int slotIdx)
    {
        TrcPidController.PidCoefficients pidCoeffs;

        switch (slotIdx)
        {
            case 0:
                recordResponseCode(
                    "getPidCoefficientsSlot0", motor.getConfigurator().refresh(talonFxConfigs.Slot0));
                pidCoeffs = new TrcPidController.PidCoefficients(
                    talonFxConfigs.Slot0.kP, talonFxConfigs.Slot0.kI, talonFxConfigs.Slot0.kD,
                    talonFxConfigs.Slot0.kV);
                break;

            case 1:
                recordResponseCode(
                    "getPidCoefficientsSlot1", motor.getConfigurator().refresh(talonFxConfigs.Slot1));
                pidCoeffs = new TrcPidController.PidCoefficients(
                    talonFxConfigs.Slot1.kP, talonFxConfigs.Slot1.kI, talonFxConfigs.Slot1.kD,
                    talonFxConfigs.Slot1.kV);
                break;

            case 2:
                recordResponseCode(
                    "getPidCoefficientsSlot2", motor.getConfigurator().refresh(talonFxConfigs.Slot2));
                pidCoeffs = new TrcPidController.PidCoefficients(
                    talonFxConfigs.Slot2.kP, talonFxConfigs.Slot2.kI, talonFxConfigs.Slot2.kD,
                    talonFxConfigs.Slot2.kV);
                break;

            default:
                pidCoeffs = null;
        }

        return pidCoeffs;
    }   //getPidCoefficients

    /**
     * This method sets the PID coefficients of the motor controller's velocity PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setMotorVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        setPidCoefficients(PIDSLOT_VELOCITY, pidCoeff);
    }   //setMotorVelocityPidCoefficients

    /**
     * This method returns the PID coefficients of the motor controller's velocity PID controller.
     *
     * @return PID coefficients of the motor's veloicty PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getMotorVelocityPidCoefficients()
    {
        return getPidCoefficients(PIDSLOT_VELOCITY);
    }   //getMotorVelocityPidCoefficients

    /**
     * This method sets the PID tolerance of the motor controller's velocity PID controller.
     *
     * @param tolerance specifies the PID tolerance to set.
     */
    @Override
    public void setMotorVelocityPidTolerance(double tolerance)
    {
        throw new UnsupportedOperationException("Controller does not support setting PID Tolerance.");
    }   //setMotorVelocityPidTolerance

    /**
     * This method checks if the motor is at the set velocity.
     *
     * @return true if motor is on target, false otherwise.
     */
    @Override
    public boolean getMotorVelocityOnTarget(double tolerance)
    {
        boolean onTarget = false;

        if (motor.getControlMode().getValue().equals(ControlModeValue.VelocityDutyCycle))
        {
            onTarget = motor.getClosedLoopError().getValueAsDouble() <= tolerance;
        }

        return onTarget;
    }   //getMotorVelocityOnTarget

    /**
     * This method sets the PID coefficients of the motor controller's position PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setMotorPositionPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        setPidCoefficients(PIDSLOT_POSITION, pidCoeff);
    }   //setMotorPositionPidCoefficients

    /**
     * This method returns the PID coefficients of the motor controller's position PID controller.
     *
     * @return PID coefficients of the motor's position PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getMotorPositionPidCoefficients()
    {
        return getPidCoefficients(PIDSLOT_POSITION);
    }   //getMotorPositionPidCoefficients

    /**
     * This method sets the PID tolerance of the motor controller's position PID controller.
     *
     * @param tolerance specifies the PID tolerance to set.
     */
    @Override
    public void setMotorPositionPidTolerance(double tolerance)
    {
        throw new UnsupportedOperationException("Controller does not support setting PID Tolerance.");
    }   //setMotorPositionPidTolerance

    /**
     * This method checks if the motor is at the set position.
     *
     * @param tolerance specifies the PID tolerance.
     * @return true if motor is on target, false otherwise.
     */
    @Override
    public boolean getMotorPositionOnTarget(double tolerance)
    {
        boolean onTarget = false;

        if (motor.getControlMode().getValue().equals(ControlModeValue.PositionDutyCycle))
        {
            onTarget = motor.getClosedLoopError().getValueAsDouble() <= tolerance;
        }

        return onTarget;
    }   //getMotorPositionOnTarget

    /**
     * This method sets the PID coefficients of the motor controller's current PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setMotorCurrentPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        throw new UnsupportedOperationException("Controller does not support current control PID coefficients.");
    }   //setMotorCurrentPidCoefficients

    /**
     * This method returns the PID coefficients of the motor controller's current PID controller.
     *
     * @return PID coefficients of the motor's current PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getMotorCurrentPidCoefficients()
    {
        throw new UnsupportedOperationException("Controller does not support current control PID coefficients.");
    }   //geteMotorCurrentPidCoefficients

    /**
     * This method sets the PID tolerance of the motor controller's current PID controller.
     *
     * @param tolerance specifies the PID tolerance to set.
     */
    @Override
    public void setMotorCurrentPidTolerance(double tolerance)
    {
        throw new UnsupportedOperationException("Controller does not support setting PID Tolerance.");
    }   //setMotorCurrentPidTolerance

    /**
     * This method checks if the motor is at the set current.
     *
     * @param tolerance specifies the PID tolerance.
     * @return true if motor is on target, false otherwise.
     */
    @Override
    public boolean getMotorCurrentOnTarget(double tolerance)
    {
        boolean onTarget = false;

        if (motor.getControlMode().getValue().equals(ControlModeValue.TorqueCurrentFOC))
        {
            onTarget = motor.getClosedLoopError().getValueAsDouble() <= tolerance;
        }

        return onTarget;
    }   //getMotorCurrentOnTarget

    //
    // The following methods override the software simulation in TrcMotor providing direct support in hardware.
    //

    /**
     * This method enables/disables voltage compensation so that it will maintain the motor output regardless of
     * battery voltage.
     *
     * @param batteryNominalVoltage specifies the nominal voltage of the battery to enable, null to disable.
     */
    @Override
    public void setVoltageCompensationEnabled(Double batteryNominalVoltage)
    {
        this.batteryNominalVoltage = batteryNominalVoltage;
    }   //setVoltageCompensationEnabled

    /**
     * This method checks if voltage compensation is enabled.
     *
     * @return true if voltage compensation is enabled, false if disabled.
     */
    @Override
    public boolean isVoltageCompensationEnabled()
    {
        return batteryNominalVoltage != null;
    }   //isVoltageCompensationEnabled

    /**
     * This method enables motion profile support.
     *
     * @param velocity specifies cruise velocity in the unit of rps.
     * @param acceleration specifies acceleration in the unit of rot per sec square.
     * @param jerk specifies acceleration derivation in the unit of rot per sec cube.
     */
    @Override
    public void enableMotionProfile(double velocity, double acceleration, double jerk)
    {
        talonFxConfigs.MotionMagic.MotionMagicCruiseVelocity = velocity;
        talonFxConfigs.MotionMagic.MotionMagicAcceleration = acceleration;
        talonFxConfigs.MotionMagic.MotionMagicJerk = jerk;
        if (recordResponseCode("setMotionMagic", motor.getConfigurator().apply(talonFxConfigs.MotionMagic)) ==
            StatusCode.OK)
        {
            useMotionProfile = true;
        }
    }   //enableMotionProfile

    /**
     * This method disables motion profile support.
     */
    @Override
    public void disableMotionProfile()
    {
        useMotionProfile = false;
    }   //disableMotionProfile

    /**
     * This method sets this motor to follow another motor.
     *
     * @param otherMotor specifies the other motor to follow.
     * @param inverted specifies true if this motor is inverted from the motor it is following, false otherwise.
     */
    @Override
    public void follow(TrcMotor otherMotor, boolean inverted)
    {
        if (otherMotor instanceof FrcCANPhoenix6Controller)
        {
            recordResponseCode(
                "follow",
                motor.setControl(new Follower(((FrcCANPhoenix6Controller<?>) otherMotor).motor.getDeviceID(), inverted)));
        }
        else
        {
            super.follow(otherMotor, inverted);
        }
    }   //follow

}   //class FrcCANPhoenix6Controller
