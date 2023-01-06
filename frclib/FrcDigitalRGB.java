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

import edu.wpi.first.wpilibj.DigitalOutput;
import TrcCommonLib.trclib.TrcRGBLight;

public class FrcDigitalRGB extends TrcRGBLight
{
    private DigitalOutput redLight;
    private DigitalOutput greenLight;
    private DigitalOutput blueLight;
    private boolean redState = false;
    private boolean greenState = false;
    private boolean blueState = false;

    public FrcDigitalRGB(
            final String instanceName, int redChannel, int greenChannel, int blueChannel)
    {
        super(instanceName);
        redLight = new DigitalOutput(redChannel);
        greenLight = new DigitalOutput(greenChannel);
        blueLight = new DigitalOutput(blueChannel);
    }   //FrcDigitalRGB

    //
    // Implements TrcRGBLight abstract methods.
    //

    public boolean getRed()
    {
        return redState;
    }   //getRed

    public boolean getGreen()
    {
        return greenState;
    }   //getGreen

    public boolean getBlue()
    {
        return blueState;
    }   //getBlue

    public void setRed(boolean enabled)
    {
        redState = enabled;
        redLight.set(enabled);
    }   //setRed

    public void setGreen(boolean enabled)
    {
        greenState = enabled;
        greenLight.set(enabled);
    }   //setGreen

    public void setBlue(boolean enabled)
    {
        blueState = enabled;
        blueLight.set(enabled);
    }   //setBlue

}   //class FrcDigitalRGB
