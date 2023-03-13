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

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import TrcCommonLib.trclib.TrcAddressableLED;
import TrcCommonLib.trclib.TrcColor;

/**
 * This class implements a platform dependent Addressable LED device. It uses the WPILib AddressableLED class
 * and provides methods to update the color pattern of the LED strip.
 */
public class FrcAddressableLED extends TrcAddressableLED
{
    private static final Color offColor = new Color(0, 0, 0);
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numLEDs      specifies the number of LEDs on the strip.
     * @param channel      specifies the PWM channel the device is on.
     */
    public FrcAddressableLED(String instanceName, int numLEDs, int channel)
    {
        super(instanceName, numLEDs);

        led = new AddressableLED(channel);
        led.setLength(numLEDs);
        ledBuffer = new AddressableLEDBuffer(numLEDs);
    }   //FrcAddressableLED

    /**
     * This method resets the LED strip by turning all LEDs off.
     */
    public void reset()
    {
        for (int i = 0; i < ledBuffer.getLength(); i++)
        {
            ledBuffer.setLED(i, offColor);
        }
        led.setData(ledBuffer);
   }   //reset

    /**
     * This method enables/disables the AddressableLED device.
     *
     * @param enabled specifies true to enable the device, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            led.start();
            reset();
        }
        else
        {
            reset();
            led.stop();
        }
    }   //setEnabled

    //
    // Implements TrcAddressableLED abstract methods.
    //

    /**
     * This method update the LED strip to the current pattern.
     */
    @Override
    public void updateLED(TrcColor[] colorPattern)
    {
        if (colorPattern == null)
        {
            reset();
        }
        else
        {
            for (int i = 0; i < ledBuffer.getLength(); i++)
            {
                if (i < colorPattern.length)
                {
                    ledBuffer.setLED(i, ((FrcColor)colorPattern[i]).color);
                }
                else
                {
                    ledBuffer.setLED(i, offColor);
                }
            }
            led.setData(ledBuffer);
        }
    }   //updateLED
}   //class FrcAddressableLED
