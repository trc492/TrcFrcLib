/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEncoder;

/**
 * This class extends CANCoder and implements the FrcEncoder interface to allow compatibility to other types of
 * encoders.
 */
public class FrcCANCoder extends CANcoder implements TrcEncoder
{
    private final TrcDbgTrace tracer;
    private final String instanceName;
    private CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();

    // The number of non-success error codes reported by the device after sending a command.
    private int errorCount = 0;
    private StatusCode lastStatus = null;

    private double scale = 1.0;
    private double offset = 0.0;
    private double zeroOffset = 0.0;
 
    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the CANCoder.
     * @param canBus specifies the CAN bus the CANCoder is on. Possible CAN bus
     *                 strings are:
     *                 <ul>
     *                   <li>"rio" for the native roboRIO CAN bus
     *                   <li>CANivore name or serial number
     *                   <li>SocketCAN interface (non-FRC Linux only)
     *                   <li>"*" for any CANivore seen by the program
     *                   <li>empty string (default) to select the default for the
     *                       system:
     *                   <ul>
     *                     <li>"rio" on roboRIO
     *                     <li>"can0" on Linux
     *                     <li>"*" on Windows
     *                   </ul>
     *                 </ul>
     */
    public FrcCANCoder(String instanceName, int canId, String canBus)
    {
        super(canId, canBus);
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        recordResponseCode("readConfigs", getConfigurator().refresh(cancoderConfigs));
    }   //FrcCANCoder

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the CANCoder.
     */
    public FrcCANCoder(String instanceName, int canId)
    {
        this(instanceName, canId, "");
    }   //FrcCANCoder

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

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
     * This method resets the motor controller configurations to factory default so that everything is at known state.
     */
    public StatusCode resetFactoryDefault()
    {
        // Create a new TalonFX config which will contain all factory default configurations and apply it.
        cancoderConfigs = new CANcoderConfiguration();
        return recordResponseCode("resetFactoryDefault", getConfigurator().apply(cancoderConfigs));
    }   //resetFactoryDefault

    /**
     * This method configures the absolute range mode of the encoder.
     *
     * @param range0To1 specifies true to set it to Unsigned_0To1 mode, false to Signed_PlusMinusHalf mode.
     */
    public StatusCode setAbsoluteRange(boolean range0To1)
    {
        cancoderConfigs.MagnetSensor.AbsoluteSensorRange =
            range0To1? AbsoluteSensorRangeValue.Unsigned_0To1: AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        return recordResponseCode("setAbsoluteRange", getConfigurator().apply(cancoderConfigs));
    }   //setAbsoluteRange

    //
    // Implements the FrcEncoder interface.
    //

    /**
     * This method resets the encoder position.
     */
    @Override
    public void reset()
    {
        throw new UnsupportedOperationException("CANCoder does not support reset.");
    }   //reset

    /**
     * This method reads the absolute position of the encoder.
     *
     * @return absolute position of the encoder. The absolute position may be unsigned (for example: [0,360) deg),
     *         or signed (for example: [-180,+180) deg). This is determined by a configuration. The default selection
     *         is unsigned.
     */
    @Override
    public double getRawPosition()
    {
        return super.getAbsolutePosition().getValueAsDouble();
    }   //getRawPosition

    /**
     * This method returns the encoder position adjusted by scale and offset.
     *
     * @return encoder position adjusted by scale and offset.
     */
    @Override
    public double getScaledPosition()
    {
        // Offset must be in the same unit as the absolute position.
        return (super.getAbsolutePosition().getValueAsDouble() - zeroOffset) * scale + offset;
    }   //getScaledPosition

    /**
     * This method reverses the direction of the encoder.
     *
     * @param inverted specifies true to reverse the encoder direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        cancoderConfigs.MagnetSensor.SensorDirection =
                inverted? SensorDirectionValue.Clockwise_Positive: SensorDirectionValue.CounterClockwise_Positive;
        recordResponseCode("setInverted", getConfigurator().apply(cancoderConfigs));
    }   //setInverted

    /**
     * This method checks if the encoder direction is inverted.
     *
     * @return true if encoder direction is rerversed, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        recordResponseCode("isInverted", getConfigurator().refresh(cancoderConfigs));
        return cancoderConfigs.MagnetSensor.SensorDirection == SensorDirectionValue.Clockwise_Positive;
    }   //isInverted

    /**
     * This method sets the encoder scale and offset.
     *
     * @param scale specifies the scale value.
     * @param offset specifies the offset value.
     * @param zeroOffset specifies the zero offset for absolute encoder.
     */
    @Override
    public void setScaleAndOffset(double scale, double offset, double zeroOffset)
    {
        this.scale = scale;
        this.offset = offset;
        this.zeroOffset = zeroOffset;
    }   //setScaleAndOffset

}   //class FrcCANCoder
