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
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidController;

public abstract class FrcCANPhoenixController<T extends BaseTalon> extends TrcMotor
{
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();

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
    private Double motorPower = null;
    private boolean revLimitSwitchInverted = false;
    private boolean fwdLimitSwitchInverted = false;
    private FeedbackDevice feedbackDeviceType = FeedbackDevice.QuadEncoder;

    // The number of non-success error codes reported by the device after sending a command.
    private int errorCount = 0;
    private ErrorCode lastError = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param baseTalon the base talon object.
     */
    public FrcCANPhoenixController(final String instanceName, T baseTalon)
    {
        super(instanceName);
        motor = baseTalon;
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
    private ErrorCode recordResponseCode(String operation, ErrorCode errorCode)
    {
        lastError = errorCode;
        if (errorCode != null && !errorCode.equals(ErrorCode.OK))
        {
            errorCount++;
            globalTracer.traceErr(instanceName + ".recordResponseCode", "%s (ErrCode=%s)", operation, errorCode);
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
        recordResponseCode("configFactoryDefault", motor.configFactoryDefault());
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
        return motor.getBusVoltage() > 0.0;
    }   //isConnected

    /**
     * This method sets this motor to follow another motor.
     *
     * @param otherMotor specifies the other motor to follow.
     */
    @Override
    public void followMotor(TrcMotor otherMotor)
    {
        if (otherMotor instanceof FrcCANPhoenixController)
        {
            // Can only follow the same type of motor natively.
            motor.follow(((FrcCANPhoenixController<?>) otherMotor).motor);
        }
        else
        {
            // Not the same type of motor, let TrcMotor simulates it.
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
        recordResponseCode("configVoltageCompSaturation", motor.configVoltageCompSaturation(batteryNominalVoltage));
        motor.enableVoltageCompensation(true);
    }   //enableVoltageCompensation

    /**
     * This method disables voltage compensation
     */
    @Override
    public void disableVoltageCompensation()
    {
        motor.enableVoltageCompensation(false);
    }   //disableVoltageCompensation

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
     * This method sets the PID coefficients of the motor's PID controller.
     *
     * @param pidCoeff specifies the PID coefficients to set.
     */
    @Override
    public void setPidCoefficients(TrcPidController.PidCoefficients pidCoeff)
    {
        recordResponseCode("config_kP", motor.config_kP(0, pidCoeff.kP));
        recordResponseCode("config_kI", motor.config_kI(0, pidCoeff.kI));
        recordResponseCode("config_kD", motor.config_kD(0, pidCoeff.kD));
        recordResponseCode("config_kF", motor.config_kF(0, pidCoeff.kF));
        recordResponseCode("config_IntegralZone", motor.config_IntegralZone(0, pidCoeff.iZone));
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
            motor.configGetParameter(ParamEnum.eProfileParamSlot_P, 0),
            motor.configGetParameter(ParamEnum.eProfileParamSlot_I, 0),
            motor.configGetParameter(ParamEnum.eProfileParamSlot_D, 0),
            motor.configGetParameter(ParamEnum.eProfileParamSlot_F, 0),
            motor.configGetParameter(ParamEnum.eProfileParamSlot_IZone, 0));
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
        revLimitSwitchInverted = inverted;
        recordResponseCode("configReverseLimitSwitchSource",
            motor.configReverseLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                inverted? LimitSwitchNormal.NormallyClosed: LimitSwitchNormal.NormallyOpen));
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
        fwdLimitSwitchInverted = inverted;
        recordResponseCode("configForwardLimitSwitchSource",
            motor.configForwardLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                inverted? LimitSwitchNormal.NormallyClosed: LimitSwitchNormal.NormallyOpen));
    }   //setFwdLimitSwitchInverted

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    @Override
    public boolean isRevLimitSwitchActive()
    {
        return revLimitSwitchInverted ^ (motor.isRevLimitSwitchClosed() == 1);
    }   //isRevLimitSwitchActive

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    @Override
    public boolean isFwdLimitSwitchActive()
    {
        return fwdLimitSwitchInverted ^ (motor.isFwdLimitSwitchClosed() == 1);
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
    @Override
    public void stopMotor()
    {
        setMotorPower(0.0);
    }   //stopMotor

    /**
     * This method sets the motor power.
     *
     * @param power specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    @Override
    public void setMotorPower(double power)
    {
        // Do this only if power is different from the previous set power.
        if (motorPower == null || motorPower != power)
        {
            motor.set(ControlMode.PercentOutput, power);
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
        motorPower = motor.getMotorOutputPercent();
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
        motor.setSensorPhase(inverted);
    }   //setPositionSensorInverted

    /**
     * This method returns the state of the position sensor direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isPositionSensorInverted()
    {
        // Is this correct?
        return motor.configGetParameter(ParamEnum.eSensorDirection, 0) > 0.0;
    }   //isPositionSensorInverted

    /**
     * This method commands the motor to go to the given position using close loop control.
     *
     * @param position specifies the motor position in raw sensor units.
     */
    @Override
    public void setMotorPosition(double position)
    {
        motor.set(ControlMode.Position, position);
        motorPower = null;
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
        return motor.getSelectedSensorPosition(0);
    }   //getMotorPosition

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
     * This method commands the motor to spin at the given velocity using close loop control.
     *
     * @param velocity specifies the motor velocity in raw sensor units per second.
     */
    @Override
    public void setMotorVelocity(double velocity)
    {
        // set takes a velocity value in sensor units per 100 msec.
        motor.set(ControlMode.Velocity, velocity/10.0);
        motorPower = null;
    }   //setMotorVelocity

    /**
     * This method returns the current motor velocity.
     *
     * @return current motor velocity in raw sensor units per sec.
     */
    @Override
    public double getMotorVelocity()
    {
        // getSelectedSensorVelocity returns value in sensor units per 100 msec.
        return motor.getSelectedSensorVelocity()*10.0;
    }   //getMotorVelocity

    /**
     * This method commands the motor to spin at the given current value using close loop control.
     *
     * @param current specifies current in amperes.
     */
    @Override
    public void setMotorCurrent(double current)
    {
        motor.set(ControlMode.Current, current);
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
        return motor.getStatorCurrent();
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
        motor.configPeakOutputReverse(revLimit);
        motor.configPeakOutputForward(fwdLimit);
    }   //setCloseLoopOutputLimits

}   //class FrcCANPhoenixController
