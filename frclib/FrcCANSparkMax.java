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

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcEncoder;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;

/**
 * This class implements a SparkMAX motor controller by REV robototics. It extends the TrcMotor class and
 * implements the abstract methods required by TrcMotor to be compatible with the TRC library.
 * Reference manual and API doc of the motor controller can be found here:
 * http://www.revrobotics.com/sparkmax-users-manual/?mc_cid=a60a44dc08&mc_eid=1935741b98#section-2-3
 * https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax
 */
public class FrcCANSparkMax extends TrcMotor
{
    private static final int PIDSLOT_POSITION = 0;
    private static final int PIDSLOT_VELOCITY = 1;
    private static final int PIDSLOT_CURRENT = 2;

    public final CANSparkMax motor;
    private final SparkPIDController pidCtrl;
    private final RelativeEncoder sparkMaxEncoder;
    private SparkLimitSwitch sparkMaxRevLimitSwitch, sparkMaxFwdLimitSwitch;
    private Double velSetpoint = null;
    private Double posSetpoint = null;
    private Double currentSetpoint = null;
    // The number of non-success error codes reported by the device after sending a command.
    private int errorCount = 0;
    private REVLibError lastError = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the device.
     * @param brushless specifies true if the motor is brushless, false otherwise.
     * @param lowerLimitSwitch specifies an external lower limit switch overriding the motor controller one.
     * @param upperLimitSwitch specifies an external upper limit switch overriding the motor controller one.
     * @param encoder specifies an external encoder overriding the motor controller one.
     */
    public FrcCANSparkMax(
        String instanceName, int canId, boolean brushless, TrcDigitalInput lowerLimitSwitch,
        TrcDigitalInput upperLimitSwitch, TrcEncoder encoder)
    {
        super(instanceName, lowerLimitSwitch, upperLimitSwitch, encoder);
        motor = new CANSparkMax(
            canId, brushless? CANSparkLowLevel.MotorType.kBrushless: CANSparkLowLevel.MotorType.kBrushed);
        pidCtrl = motor.getPIDController();
        sparkMaxEncoder = motor.getEncoder();
        sparkMaxRevLimitSwitch = sparkMaxFwdLimitSwitch = null;
    }   //FrcCANSparkMax

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the device.
     * @param brushless specifies true if the motor is brushless, false otherwise.
     */
    public FrcCANSparkMax(String instanceName, int canId, boolean brushless)
    {
        this(instanceName, canId, brushless, null, null, null);
    }   //FrcCANSparkMax

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
    public REVLibError getLastError()
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
    private REVLibError recordResponseCode(String operation, REVLibError errorCode)
    {
        lastError = errorCode;
        if (errorCode != null && !errorCode.equals(REVLibError.kOk))
        {
            errorCount++;
            tracer.traceErr(instanceName, operation + " (ErrCode=" + errorCode + ")");
        }
        return errorCode;
    }   //recordResponseCode

    /**
     * This method returns the motor type.
     *
     * @return true if the motor is brushless, false otherwise.
     */
    public boolean isBrushless()
    {
        return motor.getMotorType() == MotorType.kBrushless;
    }   //isBrushless

    //
    // Implements TrcMotorController interface.
    //

    // /**
    //  * This method is used to check if the motor controller supports close loop control natively.
    //  *
    //  * @return true if motor controller supports close loop control, false otherwise.
    //  */
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
    //     return motor.getFirmwareString() != null;
    // }   //isConnected

    /**
     * This method resets the motor controller configurations to factory default so that everything is at known state.
     */
    @Override
    public void resetFactoryDefault()
    {
        recordResponseCode("restoreFactoryDefault", motor.restoreFactoryDefaults());
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
     * @param triggerThresholdCurrent not used. SparkMax does not support this.
     * @param triggerThresholdTime not used. SparkMax does not support this.
     */
    @Override
    public void setCurrentLimit(double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime)
    {
        recordResponseCode("setSmartCurrentLimit", motor.setSmartCurrentLimit((int) currentLimit));
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
    //     recordResponseCode("setOutputRange", pidCtrl.setOutputRange(revLimit, fwdLimit));
    // }   //setCloseLoopOutputLimits

    /**
     * This method sets the close loop ramp rate.
     *
     * @param rampTime specifies the ramp time in seconds from neutral to full speed.
     */
    @Override
    public void setCloseLoopRampRate(double rampTime)
    {
        recordResponseCode("setClosedLoopRampRate", motor.setClosedLoopRampRate(rampTime));
    }   //setCloseLoopRampRate

    /**
     * This method sets the open loop ramp rate.
     *
     * @param rampTime specifies the ramp time in seconds from neutral to full speed.
     */
    @Override
    public void setOpenLoopRampRate(double rampTime)
    {
        recordResponseCode("setOpenLoopRampRate", motor.setOpenLoopRampRate(rampTime));
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
        recordResponseCode("setIdleMode", motor.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast));
    }   //setBrakeModeEnabled

    /**
     * This method enables the reverse limit switch and configures it to the specified type.
     *
     * @param normalClose specifies true as the normal close switch type, false as normal open.
     */
    @Override
    public void enableMotorRevLimitSwitch(boolean normalClose)
    {
        sparkMaxRevLimitSwitch = motor.getReverseLimitSwitch(normalClose? Type.kNormallyClosed: Type.kNormallyOpen);
        recordResponseCode("enableLimitSwitch", sparkMaxRevLimitSwitch.enableLimitSwitch(true));
    }   //enableMotorRevLimitSwitch

    /**
     * This method enables the forward limit switch and configures it to the specified type.
     *
     * @param normalClose specifies true as the normal close switch type, false as normal open.
     */
    @Override
    public void enableMotorFwdLimitSwitch(boolean normalClose)
    {
        sparkMaxFwdLimitSwitch = motor.getForwardLimitSwitch(normalClose? Type.kNormallyClosed: Type.kNormallyOpen);
        recordResponseCode("enableLimitSwitch", sparkMaxFwdLimitSwitch.enableLimitSwitch(true));
    }   //enableMotorFwdLimitSwitch

    /**
     * This method disables the reverse limit switch.
     */
    @Override
    public void disableMotorRevLimitSwitch()
    {
        recordResponseCode("enableLimitSwitch", sparkMaxRevLimitSwitch.enableLimitSwitch(false));
    }   //disableMotorRevLimitSwitch

    /**
     * This method disables the forward limit switch.
     */
    @Override
    public void disableMotorFwdLimitSwitch()
    {
        recordResponseCode("enableLimitSwitch", sparkMaxFwdLimitSwitch.enableLimitSwitch(false));
    }   //disableMotorFwdLimitSwitch

    /**
     * This method checks if the reverse limit switch is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    @Override
    public boolean isMotorRevLimitSwitchEnabled()
    {
        return sparkMaxRevLimitSwitch != null && sparkMaxRevLimitSwitch.isLimitSwitchEnabled();
    }   //isMotorRevLimitSwitchEnabled

    /**
     * This method checks if the forward limit switch is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    @Override
    public boolean isMotorFwdLimitSwitchEnabled()
    {
        return sparkMaxFwdLimitSwitch != null && sparkMaxFwdLimitSwitch.isLimitSwitchEnabled();
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
        throw new UnsupportedOperationException(
            "SparkMax does not support inverting limit switch. Use enableMotorRevLimitSwitch to configure its type.");
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
        throw new UnsupportedOperationException(
            "SparkMax does not support inverting limit switch. Use enableMotorFwdLimitSwitch to configure its type.");
    }   //setMotorFwdLimitSwitchInverted

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    @Override
    public boolean isMotorRevLimitSwitchActive()
    {
        return sparkMaxRevLimitSwitch != null && sparkMaxRevLimitSwitch.isPressed();
    }   //isMotorRevLimitSwitchClosed

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    @Override
    public boolean isMotorFwdLimitSwitchActive()
    {
        return sparkMaxFwdLimitSwitch != null && sparkMaxFwdLimitSwitch.isPressed();
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
            recordResponseCode(
                "setReverseSoftLimit", motor.setSoftLimit(SoftLimitDirection.kReverse, limit.floatValue()));
            recordResponseCode("enableReverseSoftLimit", motor.enableSoftLimit(SoftLimitDirection.kReverse, true));
        }
        else
        {
            recordResponseCode("enableReverseSoftLimit", motor.enableSoftLimit(SoftLimitDirection.kReverse, false));
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
            recordResponseCode(
                "setForwardSoftLimit", motor.setSoftLimit(SoftLimitDirection.kForward, limit.floatValue()));
            recordResponseCode("enableForwardSoftLimit", motor.enableSoftLimit(SoftLimitDirection.kForward, true));
        }
        else
        {
            recordResponseCode("enableForwardSoftLimit", motor.enableSoftLimit(SoftLimitDirection.kForward, false));
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
        recordResponseCode("encoderSetInverted", sparkMaxEncoder.setInverted(inverted));
    }   //setMotorPositionSensorInverted

    /**
     * This method returns the state of the position sensor direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isMotorPositionSensorInverted()
    {
        return sparkMaxEncoder.getInverted();
    }   //isMotorPositionSensorInverted

    /**
     * This method resets the motor position sensor, typically an encoder.
     */
    @Override
    public void resetMotorPosition()
    {
        recordResponseCode("encoderReset", sparkMaxEncoder.setPosition(0.0));
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
     * This method sets the raw motor power.
     *
     * @param power specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    @Override
    public void setMotorPower(double power)
    {
        motor.set(power);
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
        return motor.getAppliedOutput();
    }   //getMotorPower

    /**
     * This method commands the motor to spin at the given velocity using close loop control.
     *
     * @param velocity specifies the motor velocity in rotations per second.
     * @param acceleration specifies the max motor acceleration rotations per second square (not supported).
     * @param feedForward specifies feedforward in volts if voltage comp is ON, otherwise fractional unit between
     *        -1 and 1 (not supported).
     */
    @Override
    public void setMotorVelocity(double velocity, double acceleration, double feedForward)
    {
        // setVelocity takes a velocity value in RPM.
        velSetpoint = velocity;
        recordResponseCode(
            "setVelocity", pidCtrl.setReference(velocity*60.0, ControlType.kVelocity, PIDSLOT_VELOCITY));
    }   //setMotorVelocity

    /**
     * This method returns the current motor velocity.
     *
     * @return current motor velocity in rotations per second.
     */
    @Override
    public double getMotorVelocity()
    {
        return sparkMaxEncoder.getVelocity()/60.0;
    }   //getMotorVelocity

    /**
     * This method commands the motor to go to the given position using close loop control and optionally limits the
     * power of the motor movement.
     *
     * @param position specifies the position in rotations.
     * @param powerLimit specifies the maximum power output limits, can be null if not provided. If not provided, the
     *        previous set limit is applied.
     * @param velocity specifies the max motor veloicty rotations per second (not supportec).
     * @param feedForward specifies feedforward in volts if voltage comp is ON, otherwise fractional unit between
     *        -1 and 1 (not supported).
     */
    @Override
    public void setMotorPosition(double position, Double powerLimit, double velocity, double feedForward)
    {
        if (powerLimit != null)
        {
            recordResponseCode("setOutputRange", pidCtrl.setOutputRange(-powerLimit, powerLimit, PIDSLOT_POSITION));
        }
        posSetpoint = position;
        recordResponseCode("setPosition", pidCtrl.setReference(position, ControlType.kPosition, PIDSLOT_POSITION));
    }   //setMotorPosition

    /**
     * This method returns the motor position by reading the position sensor.
     *
     * @return current motor position in number of rotations.
     */
    @Override
    public double getMotorPosition()
    {
        return sparkMaxEncoder.getPosition();
    }   //getMotorPosition

    /**
     * This method commands the motor to spin at the given current value using close loop control.
     *
     * @param current specifies current in amperes.
     */
    @Override
    public void setMotorCurrent(double current)
    {
        currentSetpoint = current;
        recordResponseCode("setCurret", pidCtrl.setReference(current, ControlType.kCurrent, PIDSLOT_CURRENT));
    }   //setMotorCurrent

    /**
     * This method returns the motor current.
     *
     * @return motor current in amperes.
     */
    @Override
    public double getMotorCurrent()
    {
        return motor.getOutputCurrent();
    }   //getMotorCurrent

    /**
     * This method sets the PID coefficients of the specified slot.
     *
     * @param pidSlot specifies the PID slot.
     * @param pidCoeff specifies the PID coefficients to set.
     */
    private void setPidCoefficients(int pidSlot, TrcPidController.PidCoefficients pidCoeff)
    {
        recordResponseCode("setP", pidCtrl.setP(pidCoeff.kP, pidSlot));
        recordResponseCode("setI", pidCtrl.setI(pidCoeff.kI, pidSlot));
        recordResponseCode("setD", pidCtrl.setD(pidCoeff.kD, pidSlot));
        recordResponseCode("setFF", pidCtrl.setFF(pidCoeff.kF, pidSlot));
        recordResponseCode("setIZone", pidCtrl.setIZone(pidCoeff.iZone, pidSlot));
    }   //setPidCoefficients

    /**
     * This method sets the PID tolerance of the the specified slot.
     *
     * @param slotIdx specifies the slot index.
     * @param tolerance specifies PID tolerance.
     */
    private void setPidTolerance(int slotIdx, double tolerance)
    {
        recordResponseCode(
            "setSmartMotionAllowedClosedLoopError", pidCtrl.setSmartMotionAllowedClosedLoopError(tolerance, slotIdx));
    }   //setPidTolerance

    /**
     * This method returns the PID coefficients of the specified slot.
     *
     * @param pidSlot specifies the PID slot.
     * @return PID coefficients of the motor's PID controller.
     */
    private TrcPidController.PidCoefficients getPidCoefficients(int pidSlot)
    {
        return new TrcPidController.PidCoefficients(
            pidCtrl.getP(pidSlot), pidCtrl.getI(pidSlot), pidCtrl.getD(pidSlot), pidCtrl.getFF(pidSlot),
            pidCtrl.getIZone(pidSlot));
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
     * @param tolerance specifies the PID tolerance.
     * @return true if motor is on target, false otherwise.
     */
    @Override
    public boolean getMotorVelocityOnTarget(double tolerance)
    {
        setPidTolerance(PIDSLOT_VELOCITY, tolerance);
        return velSetpoint != null && Math.abs(velSetpoint - getMotorVelocity()) <= tolerance;
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
     * @param tolerance specifies the PID tolerance.
     * @return true if motor is on target, false otherwise.
     */
    @Override
    public boolean getMotorPositionOnTarget(double tolerance)
    {
        setPidTolerance(PIDSLOT_POSITION, tolerance);
        return posSetpoint != null && Math.abs(posSetpoint - getMotorPosition()) <= tolerance;
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
     * @param tolerance specifies the PID tolerance.
     * @return true if motor is on target, false otherwise.
     */
    @Override
    public boolean getMotorCurrentOnTarget(double tolerance)
    {
        setPidTolerance(PIDSLOT_CURRENT, tolerance);
        return currentSetpoint != null && Math.abs(currentSetpoint - getMotorCurrent()) <= tolerance;
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
            recordResponseCode("enableVoltageCompensation", motor.enableVoltageCompensation(batteryNominalVoltage));
        }
        else
        {
            recordResponseCode("disableVoltageCompensation", motor.disableVoltageCompensation());
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
        return motor.getVoltageCompensationNominalVoltage() != 0.0;
    }   //isVoltageCompensationEnabled

    /**
     * This method sets this motor to follow another motor.
     *
     * @param otherMotor specifies the other motor to follow.
     * @param inverted specifies true if this motor is inverted from the motor it is following, false otherwise.
     */
    @Override
    public void follow(TrcMotor otherMotor, boolean inverted)
    {
        if (otherMotor instanceof FrcCANSparkMax)
        {
            // Can only follow the same type of motor natively.
            recordResponseCode("follow", motor.follow(((FrcCANSparkMax) otherMotor).motor));
            setMotorInverted(otherMotor.isMotorInverted() ^ inverted);
        }
        else
        {
            super.follow(otherMotor, inverted);
        }
    }   //follow

}   //class FrcCANSparkMax
