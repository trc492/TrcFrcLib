/* 
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com) 
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
 
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPidActuator;

/**
 * This class implements a generic motor actuator. A motor actuator consists of a motor, a lower limit
 * switch and an optional upper limit switch. It creates all the necessary components for a PID controlled actuator
 * which includes a PID controller and a PID controlled actuator.
 */
public class FrcMotorActuator
{
    private TrcMotor actuatorMotor;
    private TrcPidActuator pidActuator;

    /**
     * Common initialization called by all constructors.
     *
     * @param instanceName specifies the instance name.
     * @param actuatorMotor specifies the actuator motor.
     * @param lowerLimitSw specifies the lower limit switch object.
     * @param upperLimitSw specifies the upper limit switch object.
     * @param actuatorParams specifies the parameters to set up the PID actuator.
     */
    private void commonInit(
        String instanceName, TrcMotor actuatorMotor, TrcDigitalInput lowerLimitSw, TrcDigitalInput upperLimitSw,
        TrcPidActuator.Parameters actuatorParams)
    {
        this.actuatorMotor = actuatorMotor;
        pidActuator = new TrcPidActuator(
            instanceName + ".pidActuator", actuatorMotor, lowerLimitSw, upperLimitSw, actuatorParams);
    }   //commonInit

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param actuatorMotor specifies the actuator motor.
     * @param lowerLimitSw specifies the lower limit switch object.
     * @param upperLimitSw specifies the upper limit switch object.
     * @param actuatorParams specifies the parameters to set up the PID actuator.
     */
    public FrcMotorActuator(
        String instanceName, TrcMotor actuatorMotor, TrcDigitalInput lowerLimitSw, TrcDigitalInput upperLimitSw,
        TrcPidActuator.Parameters actuatorParams)
    {
        commonInit(instanceName, actuatorMotor, lowerLimitSw, upperLimitSw, actuatorParams);
    }   //FrcMotorActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param actuatorMotor specifies the actuator motor.
     * @param lowerLimitSwChannel specifies the digital input channel for the lower limit switch, set to null
     *        if no lower limit switch.
     * @param lowerLimitSwInverted specifies true to invert lower limit switch, false otherwise.
     * @param upperLimitSwChannel specifies the digital input channel for the upper limit switch, set to null
     *        if no upper limit switch.
     * @param upperLimitSwInverted specifies true to invert upper limit switch, false otherwise.
     * @param actuatorParams specifies the parameters to set up the PID actuator.
     */
    public FrcMotorActuator(
        String instanceName, TrcMotor actuatorMotor, Integer lowerLimitSwChannel, boolean lowerLimitSwInverted,
        Integer upperLimitSwChannel, boolean upperLimitSwInverted, TrcPidActuator.Parameters actuatorParams)
    {
        FrcDigitalInput lowerLimitSwitch, upperLimitSwitch;

        if (lowerLimitSwChannel != null)
        {
            lowerLimitSwitch = new FrcDigitalInput(instanceName + ".lowerLimitSw", lowerLimitSwChannel);
            lowerLimitSwitch.setInverted(lowerLimitSwInverted);
        }
        else
        {
            lowerLimitSwitch = null;
        }

        if (upperLimitSwChannel != null)
        {
            upperLimitSwitch = new FrcDigitalInput(instanceName + ".upperLimitSw", upperLimitSwChannel);
            upperLimitSwitch.setInverted(upperLimitSwInverted);
        }
        else
        {
            upperLimitSwitch = null;
        }

        commonInit(instanceName, actuatorMotor, lowerLimitSwitch, upperLimitSwitch, actuatorParams);
    }   //FrcMotorActuator

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return pidActuator.toString();
    }   //toString

    /**
     * This method returns the PID actuator object.
     *
     * @return PID actuator object.
     */
    public TrcPidActuator getPidActuator()
    {
        return pidActuator;
    }   //getPidActuator

    /**
     * This method returns the motor object.
     *
     * @return motor object.
     */
    public TrcMotor getMotor()
    {
        return actuatorMotor;
    }   //getMotor

}   //class FrcMotorActuator
