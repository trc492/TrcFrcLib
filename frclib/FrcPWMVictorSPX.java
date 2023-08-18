/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

import TrcCommonLib.trclib.TrcEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class FrcPWMVictorSPX extends FrcPWMMotorController<PWMVictorSPX>
{
    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pwmChannel specifies the PWM channel number of the motor.
     * @param revLimitSw specifies the reverse limit switch, can be null if not provided.
     * @param fwdLimitSw specifies the forward limit switch, can be null if not provided.
     * @param encoder specifies the encoder object, can be null if not provided.
     */
    public FrcPWMVictorSPX(
        String instanceName, int pwmChannel, FrcDigitalInput revLimitSw, FrcDigitalInput fwdLimitSw, TrcEncoder encoder)
    {
        super(instanceName, new PWMVictorSPX(pwmChannel), revLimitSw, fwdLimitSw, encoder);
    }   //FrcPWMVictorSPX

}   //class FrcPWMVictorSPX
