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
import TrcCommonLib.trclib.TrcUtil;

public abstract class FrcCANPhoenixController<T extends BaseTalon> extends TrcMotor
{
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
    private double maxVelocity = 0.0;
    private boolean revLimitSwitchNormalOpen = false;
    private boolean fwdLimitSwitchNormalOpen = false;
    private double motorPower = 0.0;
    private FeedbackDevice feedbackDeviceType;

    /**
     * The number of non-success error codes reported by the device after sending a command.
     */
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
        resetMotorPosition();
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
     * @param errorCode specifies the error code returned by the motor controller.
     */
    private ErrorCode recordResponseCode(ErrorCode errorCode)
    {
        lastError = errorCode;
        if (errorCode != null && !errorCode.equals(ErrorCode.OK))
        {
            errorCount++;
            if (debugEnabled)
            {
                dbgTrace.traceErr("recordResponseCode", "ErrorCode=%s", errorCode);
            }
        }
        return errorCode;
    }   //recordResponseCode

    /**
     * This method sets the motor controller to velocity mode with the specified maximum velocity.
     *
     * @param maxVelocity     specifies the maximum velocity the motor can run, in sensor units per second.
     * @param pidCoefficients specifies the PIDF coefficients to send to the controller to use for velocity control.
     */
    @Override
    public void enableVelocityMode(double maxVelocity, TrcPidController.PidCoefficients pidCoefficients)
    {
        final String funcName = "enableVelocityMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "maxVel=%f,pidCoefficients=%s", maxVelocity,
                pidCoefficients == null ? "N/A" : pidCoefficients.toString());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.maxVelocity = maxVelocity;

        if (pidCoefficients != null)
        {
            this.motor.config_kP(0, pidCoefficients.kP);
            this.motor.config_kI(0, pidCoefficients.kI);
            this.motor.config_kD(0, pidCoefficients.kD);
            this.motor.config_kF(0, pidCoefficients.kF);
        }
    }   //enableVelocityMode

    /**
     * This method disables velocity mode returning it to power mode.
     */
    @Override
    public void disableVelocityMode()
    {
        final String funcName = "disableVelocityMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.maxVelocity = 0.0;
    }   //disableVelocityMode

    /**
     * This method sets this motor to follow another motor.
     */
    @Override
    public void followMotor(TrcMotor motor)
    {
        if (motor instanceof FrcCANPhoenixController)
        {
            this.motor.follow(((FrcCANPhoenixController<?>) motor).motor);
        }
        else
        {
            super.followMotor(motor);
        }
    }   //follow

    //
    // Overriding Phoenix specific methods.
    //

    /**
     * This method configures the forward limit switch to be normally open (i.e. active when close).
     *
     * @param normalOpen specifies true for normal open, false for normal close.
     */
    public void configFwdLimitSwitchNormallyOpen(boolean normalOpen)
    {
        final String funcName = "configFwdLimitSwitchNormallyOpen";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "normalOpen=%s", Boolean.toString(normalOpen));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        recordResponseCode(motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            normalOpen ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed, 0));
        fwdLimitSwitchNormalOpen = normalOpen;
    }   //configFwdLimitSwitchNormallyOpen

    /**
     * This method configures the reverse limit switch to be normally open (i.e. active when close).
     *
     * @param normalOpen specifies true for normal open, false for normal close.
     */
    public void configRevLimitSwitchNormallyOpen(boolean normalOpen)
    {
        final String funcName = "configRevLimitSwitchNormallyOpen";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "normalOpen=%s", Boolean.toString(normalOpen));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        recordResponseCode(motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            normalOpen ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed, 0));
        revLimitSwitchNormalOpen = normalOpen;
    }   //configRevLimitSwitchNormallyOpen

    /**
     * This method sets the feedback device type.
     *
     * @param devType specifies the feedback device type.
     */
    public void setFeedbackDevice(FeedbackDevice devType)
    {
        final String funcName = "setFeedbackDevice";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "devType=%s", devType.toString());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.feedbackDeviceType = devType;
        recordResponseCode(motor.configSelectedFeedbackSensor(devType, 0, 10));
    }   //setFeedbackDevice

    //
    // Implements TrcMotor abstract methods and overrides some of its methods supported in hardware.
    //

    /**
     * This method returns the state of the motor controller direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        final String funcName = "getInverted";
        boolean inverted = motor.getInverted();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(inverted));
        }

        return inverted;
    }   //getInverted

    /**
     * This method inverts the motor direction.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        final String funcName = "setInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setInverted(inverted);
        recordResponseCode(motor.getLastError());
    }   //setInverted

    /**
     * This method gets the last set power.
     *
     * @return the last setPower value.
     */
    @Override
    public double getMotorPower()
    {
        final String funcName = "getMotorPower";
        double power = motor.getMotorOutputPercent();
        recordResponseCode(motor.getLastError());

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", power);
        }

        return power;
    }   //getMotorPower

    /**
     * This method sets the raw motor power.
     *
     * @param power specifies the percentage power (range -1.0 to 1.0) to be set.
     */
    @Override
    public void setMotorPower(double power)
    {
        final String funcName = "setMotorPower";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "value=%f", power);
        }

        if (power != motorPower)
        {
            if (maxVelocity != 0.0)
            {
                motor.set(ControlMode.Velocity, power);
            }
            else
            {
                motor.set(ControlMode.PercentOutput, TrcUtil.round(power*maxVelocity));
            }
            recordResponseCode(motor.getLastError());
            motorPower = power;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "! (value=%f)", power);
        }
    }   //setMotorPower

    /**
     * This method resets the motor position sensor, typically an encoder. This method emulates a reset for a
     * potentiometer.
     */
    @Override
    public void resetMotorPosition()
    {
        final String funcName = "resetMotorPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        ErrorCode error = recordResponseCode(motor.setSelectedSensorPosition(0, 0, 10));
        if (error != ErrorCode.OK)
        {
            TrcDbgTrace.getGlobalTracer().traceErr(
                funcName, "resetPosition() on device %d failed with error %s!", motor.getDeviceID(), error.name());
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
        final String funcName = "getMotorPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        double currPos = motor.getSelectedSensorPosition(0);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", currPos);
        }

        return currPos;
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
        final String funcName = "getMotorVelocity";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        double currVel = motor.getSelectedSensorVelocity() / 0.1;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", currVel);
        }

        return currVel;
    }   //getMotorVelocity

    /**
     * This method returns the state of the reverse limit switch.
     *
     * @return true if reverse limit switch is active, false otherwise.
     */
    @Override
    public boolean isRevLimitSwitchActive()
    {
        final String funcName = "isRevLimitSwitchActive";
        boolean isActive = revLimitSwitchNormalOpen == (motor.isRevLimitSwitchClosed() == 1);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", isActive);
        }

        return isActive;
    }   //isRevLimitSwitchActive

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    @Override
    public boolean isFwdLimitSwitchActive()
    {
        final String funcName = "isFwdLimitSwitchActive";
        boolean isActive = fwdLimitSwitchNormalOpen == (motor.isFwdLimitSwitchClosed() == 1);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", isActive);
        }

        return isActive;
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
        final String funcName = "setBrakeModeEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
        recordResponseCode(motor.getLastError());
    }   //setBrakeModeEnabled

    /**
     * This method checks if the device is connected to the robot.
     *
     * @return True if the device is connected, false otherwise.
     */
    @Override
    public boolean isConnected()
    {
        // hacky, but should work
        return motor.getBusVoltage() > 0.0;
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
        final String funcName = "setPositionSensorInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setSensorPhase(inverted);
        recordResponseCode(motor.getLastError());
    }   //setPositionSensorInverted

}   //class FrcCANPhoenixController
