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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcEncoder;

/**
 * This class implements a CANTalon motor controller. It extends the TrcMotor class and
 * implements the abstract methods required by TrcMotor to be compatible with the TRC library.
 */
public class FrcCANTalon extends FrcCANPhoenixController<TalonSRX>
{
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private static final boolean debugEnabled = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the device.
     * @param lowerLimitSwitch specifies an external lower limit switch overriding the motor controller one.
     * @param upperLimitSwitch specifies an external upper limit switch overriding the motor controller one.
     * @param encoder specifies an external encoder overriding the motor controller one.
     */
    public FrcCANTalon(
        String instanceName, int canId, TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch,
        TrcEncoder encoder)
    {
        super(instanceName, new TalonSRX(canId), lowerLimitSwitch, upperLimitSwitch, encoder);
    }   //FrcCANTalon

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the device.
     */
    public FrcCANTalon(String instanceName, int canId)
    {
        this(instanceName, canId, null, null, null);
    }   //FrcCANTalon

    /**
     * This method synchronizes the internal encoder to the absolute zero position.
     *
     * @param rangeLow specifies the low range of the absolute encoder reading.
     * @param rangeHigh specifies the high range of the absolute encoder reading.
     * @param crossZeroOnInterval specifies true if the absoulte encoder will cross the zero point.
     * @param zeroOffset specifies the encoder offset of the absolute zero position.
     */
    public void setAbsoluteZeroOffset(int rangeLow, int rangeHigh, boolean crossZeroOnInterval, int zeroOffset)
    {
        final String funcName = "setAbsoluteZeroOffset";
        ErrorCode error;
        SensorCollection sensorCollection = motor.getSensorCollection();

        error = sensorCollection.syncQuadratureWithPulseWidth(
            rangeLow, rangeHigh, crossZeroOnInterval, -zeroOffset, 10);
        if (error != ErrorCode.OK)
        {
            globalTracer.traceErr(funcName, "syncQuadratureWithPulseWidth failed (error=%s).", error.name());
        }

        error = motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        if (error != ErrorCode.OK)
        {
            globalTracer.traceErr(funcName, "configSelectedFeedbackSensor failed (error=%s).", error.name());
        }

        if (debugEnabled)
        {
            globalTracer.traceInfo(
                funcName, "zeroOffset=%d, pwmPos=%d, quadPos=%d, selectedPos=%.3f",
                zeroOffset, sensorCollection.getPulseWidthPosition(), sensorCollection.getQuadraturePosition(),
                motor.getSelectedSensorPosition());
        }
    }   //setAbsoluteZeroOffset

}   //class FrcCANTalon
