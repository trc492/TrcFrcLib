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

import com.reduxrobotics.sensors.canandcoder.Canandcoder;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEncoder;

/**
 * This class extends CANCoder and implements the FrcEncoder interface to allow compatibility to other types of
 * encoders.
 */
public class FrcCanandcoder extends Canandcoder implements TrcEncoder
{
    private final TrcDbgTrace tracer;
    private final String instanceName;
    private double scale = 1.0;
    private double offset = 0.0;
    private double zeroOffset = 0.0;
    private double sign = 1.0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the CANCoder.
     */
    public FrcCanandcoder(String instanceName, int canId)
    {
        super(canId);
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
    }   //FrcCanandcoder

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

    //
    // Implements the FrcEncoder interface.
    //

    /**
     * This method resets the encoder position.
     */
    @Override
    public void reset()
    {
        // Reset relative position to zero. This does not affect absolute position.
        super.setPosition(0.0);
    }   //reset

    /**
     * This method reads the absolute position of the encoder.
     *
     * @return absolute position of the encoder in the range of [0.0, 1.0).
     */
    @Override
    public double getRawPosition()
    {
        return super.getAbsPosition();
    }   //getRawPosition

    /**
     * This method returns the encoder position adjusted by scale and offset.
     *
     * @return encoder position adjusted by scale and offset.
     */
    @Override
    public double getScaledPosition()
    {
        double absPos = super.getAbsPosition();
        double pos = sign * (absPos - zeroOffset) * scale + offset;
        tracer.traceDebug(
            instanceName,
            "pos=" + pos + " (absPos=" + absPos + ", offset=" + offset + ", scale=" + scale + "sign=" + sign + ")");
        // Offset must be in the same unit as the absolute position.
        return pos;
    }   //getScaledPosition

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

}   //class FrcCanandcoder
