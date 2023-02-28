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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;

/**
 * This class implements a SparkMAX motor controller by REV robototics. It extends the TrcMotor class and
 * implements the abstract methods required by TrcMotor to be compatible with the TRC library.
 * Reference manual of the motor controller can be found here:
 * http://www.revrobotics.com/sparkmax-users-manual/?mc_cid=a60a44dc08&mc_eid=1935741b98#section-2-3
 */
public class FrcCANSparkMax extends TrcMotor
{
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();

    public final CANSparkMax motor;
    private final SparkMaxPIDController pidCtrl;
    private final RelativeEncoder encoder;
    private SparkMaxLimitSwitch revLimitSwitch, fwdLimitSwitch;
    private Double motorPower = null;

    // The number of non-success error codes reported by the device after sending a command.
    private int errorCount = 0;
    private REVLibError lastError = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param deviceId     specifies the CAN ID of the device.
     * @param brushless    specifies true if the motor is brushless, false otherwise.
     */
    public FrcCANSparkMax(String instanceName, int deviceId, boolean brushless)
    {
        super(instanceName);
        motor = new CANSparkMax(deviceId,
            brushless? CANSparkMaxLowLevel.MotorType.kBrushless: CANSparkMaxLowLevel.MotorType.kBrushed);
        pidCtrl = motor.getPIDController();
        encoder = motor.getEncoder();
        revLimitSwitch = fwdLimitSwitch = null;
        resetFactoryDefault();
        resetPosition();
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
            globalTracer.traceErr(instanceName + ".recordResponseCode", "%s (ErrCode=%s)", operation, errorCode);
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

    /**
     * This method is used to check if the motor controller supports close loop control internally.
     *
     * @return true if motor controller supports close loop control, false otherwise.
     */
    public boolean supportCloseLoopControl()
    {
        return true;
    }   // supportCloseLoopControl

    /**
     * This method resets the motor controller configurations to factory default so that everything is at known state.
     */
    @Override
    public void resetFactoryDefault()
    {
        recordResponseCode("restoreFactoryDefault", motor.restoreFactoryDefaults());
    }   //resetFactoryDefault

    /**
     * This method checks if the device is connected to the robot.
     *
     * @return true if the device is connected, false otherwise.
     */
    @Override
    public boolean isConnected()
    {
        // Hacky, but should work
        return motor.getFirmwareString() != null;
    }   //isConnected

    /**
     * This method sets this motor to follow another motor.
     *
     * @param otherMotor specifies the other motor to follow.
     */
    @Override
    public void followMotor(TrcMotor otherMotor)
    {
        if (otherMotor instanceof FrcCANSparkMax)
        {
            recordResponseCode("follow", motor.follow(((FrcCANSparkMax) otherMotor).motor));
        }
        else if (otherMotor instanceof FrcCANTalon)
        {
            recordResponseCode("follow", motor.follow(
                CANSparkMax.ExternalFollower.kFollowerPhoenix, ((FrcCANTalon) otherMotor).motor.getDeviceID()));
        }
        else
        {
            // Unknow motor type, let TrcMotor simulate it.
            otherMotor.addFollowingMotor(this);
        }
    }   //followMotor

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
     * This method enables voltage compensation so that it will maintain the motor output regardless of battery
     * voltage.
     *
     * @param batteryNominalVoltage specifies the nominal voltage of the battery.
     */
    @Override
    public void enableVoltageCompensation(double batteryNominalVoltage)
    {
        recordResponseCode("enableVoltageCompensation", motor.enableVoltageCompensation(batteryNominalVoltage));
    }   //enableVoltageCompensation

    /**
     * This method disables voltage compensation
     */
    @Override
    public void disableVoltageCompensation()
    {
        recordResponseCode("disableVoltageCompensation", motor.disableVoltageCompensation());
    }   //disableVoltageCompensation

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
     * This method sets the PID coefficients of the motor's PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        recordResponseCode("setP", pidCtrl.setP(pidCoeff.kP));
        recordResponseCode("setI", pidCtrl.setI(pidCoeff.kI));
        recordResponseCode("setD", pidCtrl.setD(pidCoeff.kD));
        recordResponseCode("setFF", pidCtrl.setFF(pidCoeff.kF));
        recordResponseCode("setIZone", pidCtrl.setIZone(pidCoeff.iZone));
    }   //setPidCoefficients

    /**
     * This method returns the PID coefficients of the motor's PID controller.
     *
     * @return PID coefficients of the motor's PID controller.
     */
    @Override
    public TrcPidController.PidCoefficients getPidCoefficients()
    {
        return new TrcPidController.PidCoefficients(
            pidCtrl.getP(), pidCtrl.getI(), pidCtrl.getD(), pidCtrl.getFF(), pidCtrl.getIZone());
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
        revLimitSwitch = motor.getReverseLimitSwitch(inverted? Type.kNormallyClosed: Type.kNormallyOpen);
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
        fwdLimitSwitch = motor.getForwardLimitSwitch(inverted? Type.kNormallyClosed: Type.kNormallyOpen);
    }   //setFwdLimitSwitchInverted

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    @Override
    public boolean isRevLimitSwitchActive()
    {
        return revLimitSwitch != null && revLimitSwitch.isPressed();
    }   //isRevLimitSwitchClosed

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    @Override
    public boolean isFwdLimitSwitchActive()
    {
        return fwdLimitSwitch != null && fwdLimitSwitch.isPressed();
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
        motorPower = motor.getAppliedOutput();
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
        recordResponseCode("encoderSetInverted", encoder.setInverted(inverted));
    }   //setPositionSensorInverted

    /**
     * This method returns the state of the position sensor direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isPositionSensorInverted()
    {
        return encoder.getInverted();
    }   //isPositionSensorInverted

    /**
     * This method commands the motor to go to the given position using close loop control.
     *
     * @param position specifies the motor position in number of rotations.
     */
    @Override
    public void setMotorPosition(double position)
    {
        recordResponseCode("pidCtrlSetPosition", pidCtrl.setReference(position, ControlType.kPosition));
        motorPower = null;
    }   //setMotorPosition

    /**
     * This method returns the motor position by reading the position sensor.
     *
     * @return current motor position in number of rotations.
     */
    @Override
    public double getMotorPosition()
    {
        return encoder.getPosition();
    }   //getMotorPosition

    /**
     * This method resets the motor position sensor, typically an encoder.
     */
    @Override
    public void resetMotorPosition()
    {
        recordResponseCode("encoderReset", encoder.setPosition(0.0));
    }   //resetMotorPosition

    /**
     * This method commands the motor to spin at the given velocity using close loop control.
     *
     * @param velocity specifies the motor velocity in rotations per second.
     */
    @Override
    public void setMotorVelocity(double velocity)
    {
        recordResponseCode("pidCtrlSetVelocity", pidCtrl.setReference(velocity/60.0, ControlType.kVelocity));
        motorPower = null;
    }   //setMotorVelocity

    /**
     * This method returns the current motor velocity.
     *
     * @return current motor velocity in rotations per second.
     */
    @Override
    public double getMotorVelocity()
    {
        return encoder.getVelocity()*60.0;
    }   //getMotorVelocity

    /**
     * This method commands the motor to spin at the given current value using close loop control.
     *
     * @param current specifies current in amperes.
     */
    @Override
    public void setMotorCurrent(double current)
    {
        recordResponseCode("pidCtrlSetCurret", pidCtrl.setReference(current, ControlType.kCurrent));
        motorPower = null;
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
     * This method sets the close loop percentage output limits. By default the limits are set to the max at -1 to 1.
     * By setting a non-default limits, it effectively limits the output power of the close loop control.
     *
     * @param revLimit specifies the percentage output limit of the reverse direction.
     * @param fwdLimit specifies the percentage output limit of the forward direction.
     */
    @Override
    public void setCloseLoopOutputLimits(double revLimit, double fwdLimit)
    {
        throw new UnsupportedOperationException("SparkMax does not support close loop output limits.");
    }   //setCloseLoopOutputLimits

}   //class FrcCANSparkMax
