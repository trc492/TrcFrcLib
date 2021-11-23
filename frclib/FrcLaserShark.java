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

package frclib;

import com.cuforge.libcu.Lasershark;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDistanceSensor;

/**
 * This class implements the platform dependent Lasershark LIDAR sensor. It is a wrapper class extending the Lasershark
 * class.
 */
public class FrcLaserShark extends Lasershark implements TrcDistanceSensor
{
    private static final String moduleName = "FrcLaserShark";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param digitalInput specifies the digital input channel the sensor is on.
     */
    public FrcLaserShark(String instanceName, int digitalInput)
    {
        super(digitalInput);

        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer ?
                TrcDbgTrace.getGlobalTracer() :
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
    }   //FrcLaserShark

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
     * This method returns the ranged distance in millimeters.
     *
     * @return ranged distance in millimeters.
     */
    public double getDistanceMillimeters()
    {
        final String funcName = "getDistanceMillimeters";
        double distance = getDistanceMeters() * 1000.0;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceInfo(funcName, "%s: distanceMillimeters=%f", instanceName, distance);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", distance);
        }

        return distance;
    }   //getDistanceMillimeters

    /**
     * This method returns the ranged distance in inches.
     *
     * @return ranged distance in inches.
     */
    @Override
    public double getDistanceInches()
    {
        final String funcName = "getDistanceInches";
        double distance = super.getDistanceInches();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceInfo(funcName, "%s: distanceInches=%f", instanceName, distance);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", distance);
        }

        return distance;
    }   //getDistanceInches

}   //class FrcLaserShark
