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

import TrcCommonLib.trclib.TrcAbsoluteEncoder;
import TrcCommonLib.trclib.TrcAnalogInput.DataType;

/**
 * This class implements an Analog Absolute Encoders that implements the TrcEncoder interface to allow compatibility
 * to other types of encoders.
 */
public class FrcAnalogEncoder
{
    private final FrcAnalogInput analogInput;
    private final TrcAbsoluteEncoder absEncoder;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel specifies the analog channel for the encoder.
     * @param powerRailIs3V3 specifies true if analog power rail is 3.3V, false if 5V.
     */
    public FrcAnalogEncoder(String instanceName, int channel, boolean powerRailIs3V3)
    {
        analogInput = new FrcAnalogInput(instanceName, channel, null, powerRailIs3V3);
        absEncoder = new TrcAbsoluteEncoder(instanceName, this::getNormalizedValue, 0.0, 1.0);
    }   //FrcAnalogEncoder

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel specifies the analog channel for the encoder.
     */
    public FrcAnalogEncoder(String instanceName, int channel)
    {
        this(instanceName, channel, false);
    }   //FrcAnalogEncoder

    /**
     * This method returns the created Absolute Encoder object.
     *
     * @return absolute encoder object.
     */
    public TrcAbsoluteEncoder getAbsoluteEncoder()
    {
        return absEncoder;
    }   //getAbsoluteEncoder

    /**
     * This method reads the raw voltage from the analog input of the encoder.
     *
     * @return raw voltage of the analog encoder.
     */
    public double getRawVoltage()
    {
        return analogInput.getRawData(0, DataType.RAW_DATA).value;
    }   //getRawVoltage

    /**
     * This method reads the normalized analog input value in the range of 0.0 and 1.0.
     *
     * @return normalized value.
     */
    private double getNormalizedValue()
    {
        return analogInput.getProcessedData(0, DataType.NORMALIZED_DATA).value;
    }   //getNormalizedData

}   //class FrcAnalogEncoder
