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

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcEncoder;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;

public abstract class FrcCANPhoenixController<T extends BaseTalon> extends TrcMotor
{
    private static final int PIDSLOT_POSITION = 0;
    private static final int PIDSLOT_VELOCITY = 1;
    private static final int PIDSLOT_CURRENT = 2;

    private class EncoderInfo implements Sendable
    {
        @Override
        public void initSendable(SendableBuilder builder)
        {
            if (feedbackDeviceType != FeedbackDevice.QuadEncoder)
            {
                throw new IllegalStateException("Only QuadEncoder supported for Shuffleboard!");
            }

            builder.setSmartDashboardType("Quadrature Encoder");
            builder.addDoubleProperty("Speed", FrcCANPhoenixController.this::getVelocity, null);
            builder.addDoubleProperty("Distance", FrcCANPhoenixController.this::getPosition, null);
            builder.addDoubleProperty("DistancePerCount", () -> 1, null);
        }   //initSendable
    }   //class EncoderInfo

    public final T motor;
    private FeedbackDevice feedbackDeviceType;
    private boolean revLimitSwitchInverted;
    private boolean fwdLimitSwitchInverted;
    private double velPidTolerance;
    private double posPidTolerance;
    private double currentPidTolerance;

    // The number of non-success error codes reported by the device after sending a command.
    private int errorCount = 0;
    private ErrorCode lastError = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param baseTalon the base talon object.
     * @param lowerLimitSwitch specifies an external lower limit switch overriding the motor controller one.
     * @param upperLimitSwitch specifies an external upper limit switch overriding the motor controller one.
     * @param encoder specifies an external encoder overriding the motor controller one.
     */
    public FrcCANPhoenixController(
        String instanceName, T baseTalon, TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch,
        TrcEncoder encoder)
    {
        super(instanceName, lowerLimitSwitch, upperLimitSwitch, encoder);
        motor = baseTalon;
        readConfig();
    }   //FrcCANPhoenixController

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param baseTalon the base talon object.
     */
    public FrcCANPhoenixController(String instanceName, T baseTalon)
    {
        this(instanceName, baseTalon, null, null, null);
    }   //FrcCANPhoenixController

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
    public ErrorCode getLastError()
    {
        return lastError;
    }   //getLastError

    /**
     * This method checks for error code returned by the motor controller executing the last command. If there was
     * an error, the error count is incremented.
     *
     * @param operation specifies the operation that failed.
     * @param errorCode specifies the error code returned by the motor controller.
     */
    protected ErrorCode recordResponseCode(String operation, ErrorCode errorCode)
    {
        lastError = errorCode;
        if (errorCode != null && !errorCode.equals(ErrorCode.OK))
        {
            errorCount++;
            tracer.traceErr(instanceName, operation + " (ErrCode=" + errorCode + ")");
        }
        return errorCode;
    }   //recordResponseCode

    /**
     * This method sets the feedback device type.
     *
     * @param devType specifies the feedback device type.
     */
    public void setFeedbackDevice(FeedbackDevice devType)
    {
        feedbackDeviceType = devType;
        recordResponseCode("configSelectedFeedbackSensor", motor.configSelectedFeedbackSensor(devType));
    }   //setFeedbackDevice

    /**
     * This method reads the configuration of the phoenix controller.
     */
    private void readConfig()
    {
        feedbackDeviceType = FeedbackDevice.valueOf(motor.configGetParameter(ParamEnum.eFeedbackSensorType, 0, 10));
        fwdLimitSwitchInverted = LimitSwitchNormal.NormallyClosed == LimitSwitchNormal.valueOf(
            motor.configGetParameter(ParamEnum.eLimitSwitchNormClosedAndDis, 0, 10));
        revLimitSwitchInverted = LimitSwitchNormal.NormallyClosed == LimitSwitchNormal.valueOf(
            motor.configGetParameter(ParamEnum.eLimitSwitchNormClosedAndDis, 1, 10));
        velPidTolerance = getPidTolerance(PIDSLOT_VELOCITY);
        posPidTolerance = getPidTolerance(PIDSLOT_POSITION);
        currentPidTolerance = getPidTolerance(PIDSLOT_CURRENT);
    }   //readConfig

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
    //     return motor.getBusVoltage() > 0.0;
    // }   //isConnected

    /**
     * This method resets the motor controller configurations to factory default so that everything is at known state.
     */
    @Override
    public void resetFactoryDefault()
    {
        recordResponseCode("configFactoryDefault", motor.configFactoryDefault());
        readConfig();
    }   //resetFactoryDefault

    /**
     * This method returns the bus voltage of the motor controller.
     *
     * @return bus voltage of the motor controller.
     */
    @Override
    public double getBusVoltage()
    {
        return motor.getBusVoltage();
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
        recordResponseCode(
            "configSupplyCurrentLimit",
            motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                true, currentLimit, triggerThresholdCurrent, triggerThresholdTime), 30));
    }   //setCurrentLimit

    // /**
    //  * This method sets the close loop percentage output limits. By default the limits are set to the max at -1 to 1.
    //  * By setting a non-default limits, it effectively limits the output power of the close loop control.
    //  *
    //  * @param revLimit specifies the percentage output limit of the reverse direction.
    //  * @param fwdLimit specifies the percentage output limit of the forward direction.
    //  */
    // @Override
    // public void setCloseLoopOutputLimits(double revLimit, double fwdLimit)
    // {
    //     recordResponseCode("configPeakOutputReverse", motor.configPeakOutputReverse(Math.abs(revLimit)));
    //     recordResponseCode("configPeakOutputForward", motor.configPeakOutputForward(Math.abs(fwdLimit)));
    // }   //setCloseLoopOutputLimits

    /**
     * This method sets the close loop ramp rate.
     *
     * @param rampTime specifies the ramp time in seconds from neutral to full speed.
     */
    @Override
    public void setCloseLoopRampRate(double rampTime)
    {
        recordResponseCode("configClosedloopRamp", motor.configClosedloopRamp(rampTime));
    }   //setCloseLoopRampRate

    /**
     * This method sets the open loop ramp rate.
     *
     * @param rampTime specifies the ramp time in seconds from neutral to full speed.
     */
    @Override
    public void setOpenLoopRampRate(double rampTime)
    {
        recordResponseCode("configOpenloopRamp", motor.configOpenloopRamp(rampTime));
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
        motor.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    }   //setBrakeModeEnabled

    /**
     * This method enables the reverse limit switch and configures it to the specified type.
     *
     * @param normalClose specifies true as the normal close switch type, false as normal open.
     */
    @Override
    public void enableMotorRevLimitSwitch(boolean normalClose)
    {
        recordResponseCode(
            "configReverseLimitSwitch", motor.configReverseLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                normalClose? LimitSwitchNormal.NormallyClosed: LimitSwitchNormal.NormallyOpen));
    }   //enableMotorRevLimitSwitch

    /**
     * This method enables the forward limit switch and configures it to the specified type.
     *
     * @param normalClose specifies true as the normal close switch type, false as normal open.
     */
    @Override
    public void enableMotorFwdLimitSwitch(boolean normalClose)
    {
        recordResponseCode(
            "configForwardLimitSwitch", motor.configForwardLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                normalClose? LimitSwitchNormal.NormallyClosed: LimitSwitchNormal.NormallyOpen));
    }   //enableMotorFwdLimitSwitch

    /**
     * This method disables the reverse limit switch.
     */
    @Override
    public void disableMotorRevLimitSwitch()
    {
        recordResponseCode(
            "configReverseLimitSwitch", motor.configReverseLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled));
    }   //disableMotorRevLimitSwitch

    /**
     * This method disables the forward limit switch.
     */
    @Override
    public void disableMotorFwdLimitSwitch()
    {
        recordResponseCode(
            "configForwardLimitSwitch", motor.configForwardLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled));
    }   //disableMotorFwdLimitSwitch

    /**
     * This method checks if the reverse limit switch is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    @Override
    public boolean isMotorRevLimitSwitchEnabled()
    {
        LimitSwitchNormal limitSwitchNormal = LimitSwitchNormal.valueOf(
            motor.configGetParameter(ParamEnum.eLimitSwitchNormClosedAndDis, 1));
        return limitSwitchNormal != LimitSwitchNormal.Disabled;
    }   //isMotorRevLimitSwitchEnabled

    /**
     * This method checks if the forward limit switch is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    @Override
    public boolean isMotorFwdLimitSwitchEnabled()
    {
        LimitSwitchNormal limitSwitchNormal = LimitSwitchNormal.valueOf(
            motor.configGetParameter(ParamEnum.eLimitSwitchNormClosedAndDis, 0));
        return limitSwitchNormal != LimitSwitchNormal.Disabled;
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
        revLimitSwitchInverted = inverted;
        recordResponseCode("configReverseLimitSwitchSource",
            motor.configReverseLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                inverted? LimitSwitchNormal.NormallyClosed: LimitSwitchNormal.NormallyOpen));
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
        fwdLimitSwitchInverted = inverted;
        recordResponseCode("configForwardLimitSwitchSource",
            motor.configForwardLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                inverted? LimitSwitchNormal.NormallyClosed: LimitSwitchNormal.NormallyOpen));
    }   //setMotorFwdLimitSwitchInverted

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    @Override
    public boolean isMotorRevLimitSwitchActive()
    {
        return revLimitSwitchInverted ^ (motor.isRevLimitSwitchClosed() == 1);
    }   //isMotorRevLimitSwitchActive

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    @Override
    public boolean isMotorFwdLimitSwitchActive()
    {
        return fwdLimitSwitchInverted ^ (motor.isFwdLimitSwitchClosed() == 1);
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
            recordResponseCode("configReverseSoftLimitThreshold", motor.configReverseSoftLimitThreshold(limit));
            recordResponseCode("configReverseSoftLimitEnable", motor.configReverseSoftLimitEnable(true));
        }
        else
        {
            recordResponseCode("configReverseSoftLimitEnable", motor.configReverseSoftLimitEnable(false));
        }
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
            recordResponseCode("configForwardSoftLimitThreshold", motor.configForwardSoftLimitThreshold(limit));
            recordResponseCode("configForwardSoftLimitEnable", motor.configForwardSoftLimitEnable(true));
        }
        else
        {
            recordResponseCode("configForwardSoftLimitEnable", motor.configForwardSoftLimitEnable(false));
        }
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
        motor.setSensorPhase(inverted);
    }   //setMotorPositionSensorInverted

    /**
     * This method returns the state of the position sensor direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isMotorPositionSensorInverted()
    {
        // Is this correct?
        return motor.configGetParameter(ParamEnum.eSensorDirection, 0) > 0.0;
    }   //isMotorPositionSensorInverted

    /**
     * This method resets the motor position sensor, typically an encoder.
     */
    @Override
    public void resetMotorPosition()
    {
        if (feedbackDeviceType != FeedbackDevice.Analog)
        {
            recordResponseCode("setSelectedSensorPosition", motor.setSelectedSensorPosition(0, 0, 30));
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
     * This method sets the motor power.
     *
     * @param power specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    @Override
    public void setMotorPower(double power)
    {
        motor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, power);
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
        return motor.getMotorOutputPercent();
    }   //getMotorPower

    /**
     * This method commands the motor to spin at the given velocity using close loop control.
     *
     * @param velocity specifies the motor velocity in sensor units per second.
     */
    @Override
    public void setMotorVelocity(double velocity)
    {
        // set takes a velocity value in sensor units per 100 msec.
        motor.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, velocity/10.0);
    }   //setMotorVelocity

    /**
     * This method returns the current motor velocity.
     *
     * @return current motor velocity in sensor units per sec.
     */
    @Override
    public double getMotorVelocity()
    {
        // getSelectedSensorVelocity returns value in sensor units per 100 msec.
        return motor.getSelectedSensorVelocity()*10.0;
    }   //getMotorVelocity

    /**
     * This method commands the motor to go to the given position using close loop control.
     *
     * @param position specifies the position in sensor units.
     * @param powerLimit specifies the maximum power output limits.
     */
    @Override
    public void setMotorPosition(double position, double powerLimit)
    {
        motor.configClosedLoopPeakOutput(PIDSLOT_POSITION, powerLimit);
        motor.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, position);
    }   //setMotorPosition

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position in raw sensor units.
     */
    @Override
    public double getMotorPosition()
    {
        return motor.getSelectedSensorPosition(PIDSLOT_POSITION);
    }   //getMotorPosition

    /**
     * This method commands the motor to spin at the given current value using close loop control.
     *
     * @param current specifies current in amperes.
     */
    @Override
    public void setMotorCurrent(double current)
    {
        motor.set(com.ctre.phoenix.motorcontrol.ControlMode.Current, current);
    }   //setMotorCurrent

    /**
     * This method returns the motor current.
     *
     * @return motor current in amperes.
     */
    @Override
    public double getMotorCurrent()
    {
        return motor.getStatorCurrent();
    }   //getMotorCurrent

    /**
     * This method sets the PID coefficients of the the specified slot.
     *
     * @param slotIdx specifies the slot index.
     * @param pidCoeff specifies the PID coefficients to set.
     */
    private void setPidCoefficients(int slotIdx, TrcPidController.PidCoefficients pidCoeff)
    {
        recordResponseCode("config_kP", motor.config_kP(slotIdx, pidCoeff.kP));
        recordResponseCode("config_kI", motor.config_kI(slotIdx, pidCoeff.kI));
        recordResponseCode("config_kD", motor.config_kD(slotIdx, pidCoeff.kD));
        recordResponseCode("config_kF", motor.config_kF(slotIdx, pidCoeff.kF));
        recordResponseCode("config_iZone", motor.config_IntegralZone(slotIdx, pidCoeff.iZone));
    }   //setPidCoefficients

    /**
     * This method sets the PID tolerance of the the specified slot.
     *
     * @param slotIdx specifies the slot index.
     * @param tolerance specifies PID tolerance.
     */
    private void setPidTolerance(int slotIdx, double tolerance)
    {
        recordResponseCode("configAllowableClosedloopError", motor.configAllowableClosedloopError(slotIdx, tolerance));
        switch (slotIdx)
        {
            case PIDSLOT_POSITION:
                posPidTolerance = tolerance;
                break;

            case PIDSLOT_VELOCITY:
                velPidTolerance = tolerance;
                break;

            case PIDSLOT_CURRENT:
                currentPidTolerance = tolerance;
                break;

            default:
                break;
        }
    }   //setPidTolerance

    /**
     * This method returns the PID coefficients of the specified slot.
     *
     * @param slotIdx specifies the slot index.
     * @return PID coefficients of the specified slot.
     */
    private TrcPidController.PidCoefficients getPidCoefficients(int slotIdx)
    {
        return new TrcPidController.PidCoefficients(
            motor.configGetParameter(ParamEnum.eProfileParamSlot_P, slotIdx),
            motor.configGetParameter(ParamEnum.eProfileParamSlot_I, slotIdx),
            motor.configGetParameter(ParamEnum.eProfileParamSlot_D, slotIdx),
            motor.configGetParameter(ParamEnum.eProfileParamSlot_F, slotIdx),
            motor.configGetParameter(ParamEnum.eProfileParamSlot_IZone, slotIdx));
    }   //getPidCoefficients

    /**
     * This method returns the PID tolerance of the specified slot.
     *
     * @param slotIdx specifies the slot index.
     * @return PID tolerance of the specified slot.
     */
    private double getPidTolerance(int slotIdx)
    {
        return motor.configGetParameter(ParamEnum.eProfileParamSlot_AllowableErr, slotIdx);
    }   //getPidTolerance

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
     * This method sets the PID tolerance of the motor controller's velocity PID controller.
     *
     * @param tolerance specifies the PID tolerance to set.
     */
    @Override
    public void setMotorVelocityPidTolerance(double tolerance)
    {
        setPidTolerance(PIDSLOT_VELOCITY, tolerance);
    }   //setMotorVelocityPidTolerance

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
     * This method checks if the motor is at the set velocity.
     *
     * @return true if motor is on target, false otherwise.
     */
    @Override
    public boolean getMotorVelocityOnTarget()
    {
        return motor.getClosedLoopError(PIDSLOT_VELOCITY) <= velPidTolerance;
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
     * This method sets the PID tolerance of the motor controller's position PID controller.
     *
     * @param tolerance specifies the PID tolerance to set.
     */
    @Override
    public void setMotorPositionPidTolerance(double tolerance)
    {
        setPidTolerance(PIDSLOT_POSITION, tolerance);
    }   //setMotorPositionPidTolerance

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
     * This method checks if the motor is at the set position.
     *
     * @return true if motor is on target, false otherwise.
     */
    @Override
    public boolean getMotorPositionOnTarget()
    {
        return motor.getClosedLoopError(PIDSLOT_POSITION) <= posPidTolerance;
    }   //getMotorPositionOnTarget

    /**
     * This method sets the PID coefficients of the motor controller's current PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setMotorCurrentPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        setPidCoefficients(PIDSLOT_CURRENT, pidCoeff);
    }   //setMotorCurrentPidCoefficients

    /**
     * This method sets the PID tolerance of the motor controller's current PID controller.
     *
     * @param tolerance specifies the PID tolerance to set.
     */
    @Override
    public void setMotorCurrentPidTolerance(double tolerance)
    {
        setPidTolerance(PIDSLOT_CURRENT, tolerance);
    }   //setMotorCurrentPidTolerance

    /**
     * This method returns the PID coefficients of the motor controller's current PID controller.
     *
     * @return PID coefficients of the motor's current PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getMotorCurrentPidCoefficients()
    {
        return getPidCoefficients(PIDSLOT_CURRENT);
    }   //geteMotorCurrentPidCoefficients

    /**
     * This method checks if the motor is at the set current.
     *
     * @return true if motor is on target, false otherwise.
     */
    @Override
    public boolean getMotorCurrentOnTarget()
    {
        return motor.getClosedLoopError(PIDSLOT_CURRENT) <= currentPidTolerance;
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
        if (batteryNominalVoltage != null)
        {
            recordResponseCode("configVoltageCompSaturation", motor.configVoltageCompSaturation(batteryNominalVoltage));
            motor.enableVoltageCompensation(true);
        }
        else
        {
            motor.enableVoltageCompensation(false);
        }
    }   //setVoltageCompensationEnabled

    /**
     * This method checks if voltage compensation is enabled.
     *
     * @return true if voltage compensation is enabled, false if disabled.
     */
    @Override
    public boolean isVoltageCompensationEnabled()
    {
        return motor.isVoltageCompensationEnabled();
    }   //isVoltageCompensationEnabled

    /**
     * This method sets this motor to follow another motor.
     *
     * @param otherMotor specifies the other motor to follow.
     */
    @Override
    public void follow(TrcMotor otherMotor)
    {
        if (otherMotor instanceof FrcCANPhoenixController)
        {
            // Can only follow the same type of motor natively.
            motor.follow(((FrcCANPhoenixController<?>) otherMotor).motor);
        }
        else
        {
            super.follow(otherMotor);
        }
    }   //follow

}   //class FrcCANPhoenixController
