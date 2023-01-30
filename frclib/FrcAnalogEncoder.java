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

/**
 * This class implements an Analog Encoders by extending FrcAnalogInput and also implements the FrcEncoder interface
 * to allow compatibility to other types of encoders.
 */
public class FrcAnalogEncoder extends FrcAnalogInput implements FrcEncoder
{
    private static final double DEF_MAX_VOLTAGE = 5.0;
    private double maxVoltage;
    private boolean inverted = false;
    private double scale = 1.0;
    private double offset = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel specifies the analog channel for the encoder.
     * @param maxVoltage specifies the maximum voltage of the analog channel.
     */
    public FrcAnalogEncoder(String instanceName, int channel, double maxVoltage)
    {
        super(instanceName, channel);
        this.maxVoltage = maxVoltage;
    }   //FrcAnalogEncoder

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel specifies the analog channel for the encoder.
     */
    public FrcAnalogEncoder(String instanceName, int channel)
    {
        this(instanceName, channel, DEF_MAX_VOLTAGE);
    }   //FrcAnalogEncoder

    //
    // Implements the FrcEncoder interface.
    //

    /**
     * This method reads the raw analog input of the encoder.
     *
     * @return raw input of the encoder.
     */
    @Override
    public double getRawPosition()
    {
        double rawPos = super.getRawData(0, DataType.RAW_DATA).value;

        if (inverted)
        {
            rawPos = maxVoltage - rawPos;
        }

        return rawPos;
    }   //getRawPosition

    /**
     * This method returns the encoder position adjusted by scale and offset.
     *
     * @return encoder position adjusted by scale and offset.
     */
    @Override
    public double getPosition()
    {
        return (getRawPosition() - offset) * scale;
    }   //getPosition

    /**
     * This method reverses the direction of the encoder.
     *
     * @param inverted specifies true to reverse the encoder direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        this.inverted = inverted;
    }   //setInverted

    /**
     * This method sets the encoder scale and offset.
     *
     * @param scale specifies the scale value.
     * @param offset specifies the offset value.
     */
    @Override
    public void setScaleAndOffset(double scale, double offset)
    {
        this.scale = scale;
        this.offset = offset;
    }   //setScaleAndOffset

}   //class FrcAnalogEncoder
