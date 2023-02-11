/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import TrcCommonLib.trclib.TrcAnalogInput;
import TrcCommonLib.trclib.TrcFilter;
import TrcCommonLib.trclib.TrcTimer;

/**
 * This class implements a platform dependent AnalogInput sensor extending TrcAnalogInput. It provides implementation
 * of the abstract methods in TrcAnalogInput.
 */
public class FrcAnalogInput extends TrcAnalogInput
{
    private final boolean powerRailIs3V3;
    private final AnalogInput sensor;
    private double sensorData;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel specifies the analog input channel.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data. If no filter
     *                is used, this can be set to null.
     * @param powerRailIs3V3 specifies true if analog power rail is 3.3V, false if 5V.
     */
    public FrcAnalogInput(String instanceName, int channel, TrcFilter[] filters, boolean powerRailIs3V3)
    {
        super(instanceName, 1, 0, filters);
        this.powerRailIs3V3 = powerRailIs3V3;
        sensor = new AnalogInput(channel);
    }   //FrcAnalogInput

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel specifies the analog input channel.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data. If no filter
     *                is used, this can be set to null.
     */
    public FrcAnalogInput(String instanceName, int channel, TrcFilter[] filters)
    {
        this(instanceName, channel, filters, false);
    }   //FrcAnalogInput

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param channel specifies the analog input channel.
     */
    public FrcAnalogInput(String instanceName, int channel)
    {
        this(instanceName, channel, null, false);
    }   //FrcAnalogInput

    /**
     * This method calibrates the sensor.
     */
    public void calibrate()
    {
        calibrate(DataType.INPUT_DATA);
    }   //calibrate

    //
    // Implements TrcAnalogInput abstract methods.
    //

    /**
     * This method returns the raw sensor data of the specified type.
     *
     * @param index specifies the data index (not used).
     * @param dataType specifies the data type.
     * @return raw sensor data of the specified type.
     */
    @Override
    public SensorData<Double> getRawData(int index, DataType dataType)
    {
        SensorData<Double> data;

        if (getInputElapsedTimer != null) getInputElapsedTimer.recordStartTime();
        if (dataType == DataType.RAW_DATA || dataType == DataType.INPUT_DATA)
        {
            sensorData = sensor.getVoltage();
        }
        else if (dataType == DataType.NORMALIZED_DATA)
        {
            double maxVoltage = powerRailIs3V3? RobotController.getVoltage3V3(): RobotController.getVoltage5V();
            sensorData = sensor.getVoltage()/maxVoltage;
        }
        else
        {
            throw new UnsupportedOperationException(
                "AnalogInput sensor only support RAW_DATA/INPUT_DATA/NORMALIZED_DATA types.");
        }
        if (getInputElapsedTimer != null) getInputElapsedTimer.recordEndTime();
        data = new SensorData<>(TrcTimer.getCurrentTime(), sensorData);

        return data;
    }   //getRawData

}   //class FrcAnalogInput
