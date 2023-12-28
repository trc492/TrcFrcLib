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

import TrcCommonLib.trclib.TrcEncoder;
import edu.wpi.first.wpilibj.Encoder;

/**
 * This interface specifies a common implementation of a generic encoder with which makes different tpes of encoders
 * compatible with each other.
 */
public class FrcEncoder extends Encoder implements TrcEncoder
{
    private double sign = 1.0;
    private double scale = 1.0;
    private double offset = 0.0;
    private double zeroOffset = 0.0;

    public FrcEncoder(int channelA, int channelB, EncodingType encodingType)
    {
        super(channelA, channelB, false, encodingType);
    }   //FrcEncoder

    //
    // Implements TrcEncoder interface.
    //

    /**
     * This method reads the absolute position of the encoder.
     *
     * @return absolute position of the encoder.
     */
    @Override
    public double getRawPosition()
    {
        return get();
    }   //getRawPosition

    /**
     * This method returns the encoder position adjusted by scale and offset.
     *
     * @return encoder position adjusted by scale and offset.
     */
    @Override
    public double getPosition()
    {
        return sign * (get() - zeroOffset) * scale + offset;
    }   //getPosition

    /**
     * This method reverses the direction of the encoder.
     *
     * @param inverted specifies true to reverse the encoder direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        sign = inverted ? -1.0 : 1.0;
    }   //setInverted

    /**
     * This method checks if the encoder direction is inverted.
     *
     * @return true if encoder direction is rerversed, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        return sign < 0.0;
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

}   //interface FrcEncoder
