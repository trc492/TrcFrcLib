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
import com.revrobotics.CANSparkMax.IdleMode;
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
    private boolean brushless;
    public CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkMaxLimitSwitch fwdLimitSwitch, revLimitSwitch;
    private double currPower = 0.0;
    // private boolean feedbackDeviceIsPot = false;
    // private FeedbackDevice feedbackDeviceType;

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
        this.brushless = brushless;
        motor = new CANSparkMax(deviceId,
            brushless ? CANSparkMaxLowLevel.MotorType.kBrushless : CANSparkMaxLowLevel.MotorType.kBrushed);
        encoder = motor.getEncoder();
        fwdLimitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        revLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        resetPosition(true);
    }   //FrcCANSparkMax

    /**
     * This method sets this motor to follow another motor. If the subclass is not capable of following another motor,
     * it should throw an UnsupportedOperationException.
     *
     * @throws UnsupportedOperationException if hardware does not support it.
     */
    @Override
    public void followMotor(TrcMotor motor)
    {
        if (motor instanceof FrcCANSparkMax)
        {
            FrcCANSparkMax sparkMax = (FrcCANSparkMax) motor;
            this.motor.follow(sparkMax.motor);
        }
        else if (motor instanceof FrcCANTalon)
        {
            FrcCANTalon talon = (FrcCANTalon) motor;
            this.motor.follow(CANSparkMax.ExternalFollower.kFollowerPhoenix, talon.motor.getDeviceID());
        }
        else
        {
            // Unknown motor type, add this motor to the follow list of the other motor and let TrcMotor simulates
            // motor following.
            motor.addFollowingMotor(this);
        }
    }   //follow

    /**
     * This method returns the motor type.
     *
     * @return true if the motor is brushless, false otherwise.
     */
    public boolean isBrushless()
    {
        final String funcName = "isBrushless";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", brushless);
        }

        return brushless;
    }   //isBrushless

    /**
     * This method sets the motor controller to velocity mode with the specified maximum velocity.
     *
     * @param maxVelocity     specifies the maximum velocity the motor can run, in sensor units per second.
     * @param pidCoefficients specifies the PIDF coefficients to send to the Talon to use for velocity control.
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

        if (pidCoefficients != null)
        {
            SparkMaxPIDController pidController = motor.getPIDController();
            pidController.setP(pidCoefficients.kP);
            pidController.setI(pidCoefficients.kI);
            pidController.setD(pidCoefficients.kD);
            pidController.setFF(pidCoefficients.kF);
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
    }   //disableVelocityMode

    //
    // Overriding TrcMotor specific methods.
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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "normalOpen=%s", normalOpen);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        fwdLimitSwitch = motor.getForwardLimitSwitch(
            normalOpen ? Type.kNormallyOpen : Type.kNormallyClosed);
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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "normalOpen=%s", normalOpen);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        revLimitSwitch = motor.getReverseLimitSwitch(
            normalOpen ? Type.kNormallyOpen : Type.kNormallyClosed);
    }   //configRevLimitSwitchNormallyOpen

    // /**
    //  * This method sets the feedback device type.
    //  *
    //  * @param devType specifies the feedback device type.
    //  */
    // public void setFeedbackDevice(FeedbackDevice devType)
    // {
    //     final String funcName = "setFeedbackDevice";

    //     if (debugEnabled)
    //     {
    //         dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "devType=%s", devType.toString());
    //         dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
    //     }

    //     this.feedbackDeviceType = devType;
    //     recordResponseCode(motor.configSelectedFeedbackSensor(devType, 0, 0));
    //     feedbackDeviceIsPot = devType == FeedbackDevice.Analog;
    // }   //setFeedbackDevice

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
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", inverted);
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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "inverted=%s", inverted);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setInverted(inverted);
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
        double power = motor.getAppliedOutput();

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

        if (power != currPower)
        {
            motor.set(power);
            currPower = power;
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

        REVLibError error = encoder.setPosition(0.0);
        if (error != REVLibError.kOk)
        {
            TrcDbgTrace.getGlobalTracer().traceErr(funcName, "resetPosition() on SparkMax %d failed with error %s!", motor.getDeviceId(),
                error.name());
        }
    }   //resetMotorPosition

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position.
     */
    @Override
    public double getMotorPosition()
    {
        final String funcName = "getMotorPosition";
        double pos = encoder.getPosition();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%.0f", pos);
        }

        return pos;
    }   //getMotorPosition

    /**
     * This method returns the motor velocity from the platform dependent motor hardware. If the hardware does
     * not support velocity info, it should throw an UnsupportedOperationException.
     *
     * @return current motor velocity in native unit per second (RPS).
     */
    @Override
    public double getMotorVelocity()
    {
        final String funcName = "getMotorVelocity";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        double currVel = encoder.getVelocity() / 60.0;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%.0f", currVel);
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
        boolean isActive = revLimitSwitch.isPressed();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", isActive);
        }

        return isActive;
    }   //isRevLimitSwitchClosed

    /**
     * This method returns the state of the forward limit switch.
     *
     * @return true if forward limit switch is active, false otherwise.
     */
    @Override
    public boolean isFwdLimitSwitchActive()
    {
        final String funcName = "isFwdLimitSwitchActive";
        boolean isActive = fwdLimitSwitch.isPressed();

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
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", enabled);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        motor.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    }   //setBrakeModeEnabled

    /**
     * This method checks if the SparkMax is connected to the robot. This does NOT say anything about the connection
     * status of the motor.
     *
     * @return True if the SparkMax is connected, false otherwise.
     */
    @Override
    public boolean isConnected()
    {
        // hacky, but should work
        return motor.getFirmwareString() != null;
    }   //isConnected

}   //class FrcCANSparkMax
