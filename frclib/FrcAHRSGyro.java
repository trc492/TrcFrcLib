/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.kauailabs.navx.frc.AHRS;

import TrcCommonLib.trclib.TrcGyro;
import TrcCommonLib.trclib.TrcTimer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.SPI.Port;

public class FrcAHRSGyro extends TrcGyro
{
    private class GyroInfo implements Sendable
    {
        @Override
        public void initSendable(SendableBuilder builder)
        {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", () -> getZHeading().value, null);
        }   //initSendable

    }   //class GyroInfo

    public AHRS ahrs;
    private double xSign = 1.0;
    private double ySign = 1.0;
    private double zSign = 1.0;

    public FrcAHRSGyro(final String instanceName, Port port)
    {
        super(instanceName, 3, GYRO_HAS_X_AXIS | GYRO_HAS_Y_AXIS | GYRO_HAS_Z_AXIS, null);
        this.ahrs = new AHRS(port);
    }   //FrcAHRSGyro

    /**
     * This method creates a GyroInfo object and returns it.
     *
     * @return created GyroInfo object.
     */
    public Sendable getGyroSendable()
    {
        GyroInfo gyroInfo = new GyroInfo();
        SendableRegistry.setName(gyroInfo, toString());
        return gyroInfo;
    }   //getGyroSendable

    //
    // Implements TrcGyro abstract methods.
    //

    /**
     * This method returns the raw data of the specified type for the x-axis which is not supported.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the x-axis.
     */
    public SensorData getRawXData(DataType dataType)
    {
        double value = 0.0;

        if (dataType == DataType.ROTATION_RATE)
        {
            value = ahrs.getRawGyroX();
        }
        else if (dataType == DataType.HEADING)
        {
            value = ahrs.getPitch();
        }
        SensorData data = new SensorData(TrcTimer.getCurrentTime(), value);

        return data;
    }   //getRawXData

    /**
     * This method returns the raw data of the specified type for the y-axis which is not supported.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the y-axis.
     */
    public SensorData getRawYData(DataType dataType)
    {
        double value = 0.0;

        if (dataType == DataType.ROTATION_RATE)
        {
            value = ahrs.getRawGyroY();
        }
        else if (dataType == DataType.HEADING)
        {
            value = ahrs.getRoll();
        }
        SensorData data = new SensorData(TrcTimer.getCurrentTime(), value);

        return data;
    }   //getRawYData

    /**
     * This method returns the raw data of the specified type for the z-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the z-axis.
     */
    public SensorData getRawZData(DataType dataType)
    {
        double value = 0.0;

        if (dataType == DataType.ROTATION_RATE)
        {
            value = ahrs.getRate();
        }
        else if (dataType == DataType.HEADING)
        {
            value = ahrs.getAngle();
        }
        SensorData data = new SensorData(TrcTimer.getCurrentTime(), value);

        return data;
    }   //getRawZData

    /**
     * This method inverts the x-axis. This is useful if the orientation of the gyro x-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert x-axis, false otherwise.
     */
    public void setXInverted(boolean inverted)
    {
        xSign = inverted? -1.0: 1.0;
    }   //setXInverted

    /**
     * This method inverts the y-axis. This is useful if the orientation of the gyro y-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert y-axis, false otherwise.
     */
    public void setYInverted(boolean inverted)
    {
        ySign = inverted? -1.0: 1.0;
    }   //setYInverted

    /**
     * This method inverts the z-axis. This is useful if the orientation of the gyro z-axis is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert z-axis, false otherwise.
     */
    public void setZInverted(boolean inverted)
    {
        zSign = inverted? -1.0: 1.0;
    }   //setZInverted

    /**
     * This method returns the rotation rate on the x-axis.
     *
     * @return X rotation rate.
     */
    public SensorData getXRotationRate()
    {
        return new SensorData(TrcTimer.getCurrentTime(), xSign*ahrs.getRawGyroX());
    }   //getXRotationRate

    /**
     * This method returns the rotation rate on the y-axis.
     *
     * @return Y rotation rate.
     */
    public SensorData getYRotationRate()
    {
        return new SensorData(TrcTimer.getCurrentTime(), ySign*ahrs.getRawGyroY());
    }   //getYRotationRate

    /**
     * This method returns the rotation rate on the z-axis.
     *
     * @return Z rotation rate.
     */
    public SensorData getZRotationRate()
    {
        return new SensorData(TrcTimer.getCurrentTime(), zSign*ahrs.getRate());
    }   //getZRotationRate

    /**
     * This method returns the heading of the x-axis. If there is an integrator, we call the integrator to get
     * the heading. Else if we have a CardinalConverter, we call the converter to get the heading else we call
     * the platform dependent gyro to get the raw heading value.
     *
     * @return X heading.
     */
    public SensorData getXHeading()
    {
        return new SensorData(TrcTimer.getCurrentTime(), xSign*ahrs.getPitch());
    }   //getXHeading

    /**
     * This method returns the heading of the y-axis. If there is an integrator, we call the integrator to get
     * the heading. Else if we have a CardinalConverter, we call the converter to get the heading else we call
     * the platform dependent gyro to get the raw heading value.
     *
     * @return Y heading.
     */
    public SensorData getYHeading()
    {
        return new SensorData(TrcTimer.getCurrentTime(), ySign*ahrs.getRoll());
    }   //getYHeading

    /**
     * This method returns the heading of the z-axis. If there is an integrator, we call the integrator to get
     * the heading. Else if we have a CardinalConverter, we call the converter to get the heading else we call
     * the platform dependent gyro to get the raw heading value.
     *
     * @return Z heading.
     */
    public SensorData getZHeading()
    {
        return new SensorData(TrcTimer.getCurrentTime(), zSign*ahrs.getAngle());
    }   //getZHeading

    /**
     * This method resets the integrator on the x-axis.
     */
    public void resetXIntegrator()
    {
        throw new UnsupportedOperationException("Gyro does not support x-axis integrator.");
    }   //resetXIntegrator

    /**
     * This method resets the integrator on the y-axis.
     */
    public void resetYIntegrator()
    {
        throw new UnsupportedOperationException("Gyro does not support y-axis integrator.");
    }   //resetYIntegrator

    /**
     * This method resets the integrator on the z-axis.
     */
    public void resetZIntegrator()
    {
        ahrs.reset();
    }   //resetZIntegrator

}   //class FrcAHRSGyro
