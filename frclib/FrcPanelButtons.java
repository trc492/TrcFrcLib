/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
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
 * This class implements the platform dependent joystick. It provides monitoring of the joystick buttons. If the
 * caller of this class provides a button notification handler, it will call it when there are button events.
 */
public class FrcPanelButtons extends FrcJoystick
{
    //
    // Generic USB Button Panel:
    // UsagePage=0x01, Usage=0x04 (i have no idea what this means)
    //
    public enum Button
    {
        RED1(0),
        GREEN1(1),
        BLUE1(2),
        YELLOW1(3),
        WHITE1(4),
        RED2(5),
        GREEN2(6),
        BLUE2(7),
        YELLOW2(8),
        WHITE2(9);

        int value;

        Button(int value)
        {
            this.value = value;
        }   //Button

    }   //enum Button

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port         specifies the joystick port ID.
     */
    public FrcPanelButtons(String instanceName, int port)
    {
        super(instanceName, port);
    }   //FrcPanelButtons

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName      specifies the instance name.
     * @param port              specifies the joystick port ID.
     * @param deadbandThreshold specifies the deadband of the analog sticks.
     */
    public FrcPanelButtons(String instanceName, int port, double deadbandThreshold)
    {
        super(instanceName, port, deadbandThreshold);
    }   //FrcPanelButtons

}   //class FrcPanelButtons

