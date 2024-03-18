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
public class FrcDualActionJoystick extends FrcJoystick
{
    //
    // Logitech DualAction Game Controller:
    // UsagePage=0x01, Usage=0x04
    //
    public enum Button
    {
        BUTTON_X(0),
        BUTTON_A(1),
        BUTTON_B(2),
        BUTTON_Y(3),
        BUTTON_LB(4),
        BUTTON_RB(5),
        BUTTON_LT(6),
        BUTTON_RT(7),
        BUTTON_BACK(8),
        BUTTON_START(9),
        BUTTON_LTOP(10),
        BUTTON_RTOP(11);

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
    public FrcDualActionJoystick(String instanceName, int port)
    {
        super(instanceName, port);
    }   //FrcDualActionJoystick

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName      specifies the instance name.
     * @param port              specifies the joystick port ID.
     * @param deadbandThreshold specifies the deadband of the analog sticks.
     */
    public FrcDualActionJoystick(String instanceName, int port, double deadbandThreshold)
    {
        super(instanceName, port, deadbandThreshold);
    }   //FrcDualActionJoystick

}   //class FrcDualActionJoystick
