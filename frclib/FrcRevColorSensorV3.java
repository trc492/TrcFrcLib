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

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import TrcCommonLib.trclib.TrcDbgTrace;

/**
 * This class implements the platform dependent Rev Color Sensor V3. It is a wrapper class extending the ColorSensorV3
 * class.
 */
public class FrcRevColorSensorV3 extends ColorSensorV3
{
    private static final String moduleName = "FrcRevColorSensorV3";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static final I2C.Port DEF_I2C_PORT = I2C.Port.kOnboard;

    private final String instanceName;
    private final ColorMatch colorMatcher;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param i2cPort specifies the I2C port the sensor is connected to.
     */
    public FrcRevColorSensorV3(String instanceName, I2C.Port i2cPort)
    {
        super(i2cPort);

        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer ?
                TrcDbgTrace.getGlobalTracer() :
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        colorMatcher = new ColorMatch();
    }   //FrcRevColorSensorV3

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FrcRevColorSensorV3(String instanceName)
    {
        this(instanceName, DEF_I2C_PORT);
    }   //FrcRevColorSensorV3

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
     * This method adds a color to the set the color matcher will match.
     *
     * @param color specifies the color to be added to the color matcher set.
     */
    public void addColorMatch(Color color)
    {
        final String funcName = "addColorMatch";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "color=%s", color);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        colorMatcher.addColorMatch(color);
    }   //addColorMatch

    /**
     * This method returns the color read from the sensor.
     *
     * @return color read from the sensor.
     */
    @Override
    public Color getColor()
    {
        final String funcName = "getColor";
        Color color = super.getColor();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", color);
        }

        return color;
    }   //getColor

    /**
     * This method returns the Red value read from the sensor.
     *
     * @return Red value read from the sensor.
     */
    @Override
    public int getRed()
    {
        final String funcName = "getRed";
        int value = super.getRed();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", value);
        }

        return value;
    }   //getRed

    /**
     * This method returns the Green value read from the sensor.
     *
     * @return Green value read from the sensor.
     */
    @Override
    public int getGreen()
    {
        final String funcName = "getGreen";
        int value = super.getGreen();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", value);
        }

        return value;
    }   //getGreen

    /**
     * This method returns the Blue value read from the sensor.
     *
     * @return Blue value read from the sensor.
     */
    @Override
    public int getBlue()
    {
        final String funcName = "getBlue";
        int value = super.getBlue();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", value);
        }

        return value;
    }   //getBlue

    /**
     * This method returns the IR value read from the sensor.
     *
     * @return IR value read from the sensor.
     */
    @Override
    public int getIR()
    {
        final String funcName = "getIR";
        int value = super.getIR();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", value);
        }

        return value;
    }   //getIR

    /**
     * This method returns the proximity value read from the sensor.
     *
     * @return proximity value read from the sensor.
     */
    @Override
    public int getProximity()
    {
        final String funcName = "getProximity";
        int value = super.getProximity();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", value);
        }

        return value;
    }   //getProximity

    /**
     * This method returns the closest color matched in the color matcher set.
     *
     * @return matched color.
     */
    public Color getMatchedColor()
    {
        final String funcName = "getMatchedColor";
        Color color = colorMatcher.matchClosestColor(getColor()).color;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", color);
        }

        return color;
    }   //getMatchedColor

    /**
     * This methoe returns the confidence level of the matched color.
     *
     * @return match confidence level.
     */
    public double getMatchedConfidence()
    {
        final String funcName = "getMatchedConfidence";
        double confidence = colorMatcher.matchClosestColor(getColor()).confidence;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", confidence);
        }

        return confidence;
    }   //getMatchedConfidence

}   //class FrcRevColorSensorV3
