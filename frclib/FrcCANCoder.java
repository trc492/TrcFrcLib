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

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoder;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEncoder;

/**
 * This class extends CANCoder and implements the FrcEncoder interface to allow compatibility to other types of
 * encoders.
 */
public class FrcCANCoder extends CANCoder implements TrcEncoder
{
    private final String instanceName;
    private double scale = 1.0;
    private double offset = 0.0;
    private double zeroOffset = 0.0;
 
    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the CANCoder.
     */
    public FrcCANCoder(String instanceName, int canId)
    {
        super(canId);
        this.instanceName = instanceName;
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
        return super.getAbsolutePosition();
    }   //getRawPosition

    /**
     * This method returns the encoder position adjusted by scale and offset.
     *
     * @return encoder position adjusted by scale and offset.
     */
    @Override
    public double getPosition()
    {
        // Offset must be in the same unit as the absolute position.
        return (super.getAbsolutePosition() - zeroOffset) * scale + offset;
    }   //getPosition

    /**
     * This method reverses the direction of the encoder.
     *
     * @param inverted specifies true to reverse the encoder direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        final String funcName = "setInverted";
        // Configure the sensor direction to match the steering motor direction.
        ErrorCode errCode = super.configSensorDirection(inverted, 30);

        if (errCode != ErrorCode.OK)
        {
            TrcDbgTrace.globalTraceWarn(
                funcName, "%s: CANcoder.configSensorDirection failed (code=%s).", instanceName, errCode);
        }
    }   //setInverted

    /**
     * This method checks if the encoder direction is inverted.
     *
     * @return true if encoder direction is rerversed, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        return super.configGetSensorDirection();
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
