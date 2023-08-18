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

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcEncoder;

public class FrcCANFalcon extends FrcCANPhoenixController<TalonFX>
{
    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the device.
     * @param revLimitSwitch specifies an external reverse limit switch overriding the motor controller one.
     * @param fwdLimitSwitch specifies an external forward limit switch overriding the motor controller one.
     * @param encoder specifies an external encoder overriding the motor controller one.
     */
    public FrcCANFalcon(
        String instanceName, int canId, TrcDigitalInput revLimitSwitch, TrcDigitalInput fwdLimitSwitch,
        TrcEncoder encoder)
    {
        super(instanceName, new TalonFX(canId), revLimitSwitch, fwdLimitSwitch, encoder);
    }   //FrcCANFalcon

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canId specifies the CAN ID of the device.
     */
    public FrcCANFalcon(String instanceName, int canId)
    {
        this(instanceName, canId, null, null, null);
    }   //FrcCANFalcon

}   //class FrcCANFalcon
