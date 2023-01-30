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
    private double scale = 1.0;
    private double offset = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel specifies the analog channel for the encoder.
     */
    public FrcAnalogEncoder(String instanceName, int channel)
    {
        super(instanceName, channel);
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
        return super.getRawData(0, DataType.RAW_DATA).value;
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
