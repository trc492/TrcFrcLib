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
 
import java.util.Locale;

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
    /**
     * This class contains all the parameters related to the actuator motor.
     */
    public static class MotorParams
    {
        public boolean motorInverted;
        public int lowerLimitSwitchDigitalChannel;
        public boolean lowerLimitSwitchInverted;
        public int upperLimitSwitchDigitalChannel;
        public boolean upperLimitSwitchInverted;
        public boolean lowerLimitZeroCalibrateOnly;
        public double batteryNominalVoltage;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param motorInverted specifies true if actuator motor direction is inverted, false otherwise.
         * @param lowerLimitSwitchDigitalChannel specifies the digital channel for the lower limit swtich, set to -1
         *        if there is no lower limit switch.
         * @param lowerLimitSwitchInverted specifies true if lower limit switch is inverted, false otherwise.
         * @param upperLimitSwitchDigitalChannel specifies the digital channel for the upper limit switch, set to -1
         *        if there is no upper limit switch.
         * @param upperLimitSwitchInverted specifies true if upper limit switch is inverted, false otherwise.
         * @param lowerLimitZeroCalibrateOnly specifies true if lower limit switch is for zero calibration only.
         * @param batteryNominalVoltage specifies the battery nominal voltage for enabling voltage compensation,
         *        set to 0 to disable voltage compensation.
         */
        public MotorParams(
            boolean motorInverted, int lowerLimitSwitchDigitalChannel, boolean lowerLimitSwitchInverted,
            int upperLimitSwitchDigitalChannel, boolean upperLimitSwitchInverted,
            boolean lowerLimitZeroCalibrateOnly, double batteryNominalVoltage)
        {
            this.motorInverted = motorInverted;
            this.lowerLimitSwitchDigitalChannel = lowerLimitSwitchDigitalChannel;
            this.lowerLimitSwitchInverted = lowerLimitSwitchInverted;
            this.upperLimitSwitchDigitalChannel = upperLimitSwitchDigitalChannel;
            this.upperLimitSwitchInverted = upperLimitSwitchInverted;
            this.lowerLimitZeroCalibrateOnly = lowerLimitZeroCalibrateOnly;
            this.batteryNominalVoltage = batteryNominalVoltage;
        }   //MotorParams

        /**
         * Constructor: Create an instance of the object.
         *
         * @param motorInverted specifies true if actuator motor direction is inverted, false otherwise.
         * @param lowerLimitSwitchDigitalChannel specifies the digital channel for the lower limit swtich, set to -1
         *        if there is no lower limit switch.
         * @param lowerLimitSwitchInverted specifies true if lower limit switch is inverted, false otherwise.
         * @param upperLimitSwitchDigitalChannel specifies the digital channel for the upper limit switch, set to -1
         *        if there is no upper limit switch.
         * @param upperLimitSwitchInverted specifies true if upper limit switch is inverted, false otherwise.
         */
        public MotorParams(
            boolean motorInverted, int lowerLimitSwitchDigitalChannel, boolean lowerLimitSwitchInverted,
            int upperLimitSwitchDigitalChannel, boolean upperLimitSwitchInverted)
        {
            this(motorInverted, lowerLimitSwitchDigitalChannel, lowerLimitSwitchInverted,
                 upperLimitSwitchDigitalChannel, upperLimitSwitchInverted, false, 0.0);
        }   //MotorParams

        /**
         * This method returns the string format of the motorParams info.
         *
         * @return string format of the motor param info.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "motorInverted=%s,lowerLimitSWChannel=%d,lowerLimitInverted=%s," +
                "upperLimitSWChannel=%d,upperLimitInverted=%s,lowerLimitZeroCalOnly=%s,battNominalVolt=%.1f",
                motorInverted, lowerLimitSwitchDigitalChannel, lowerLimitSwitchInverted,
                upperLimitSwitchDigitalChannel, upperLimitSwitchInverted,
                lowerLimitZeroCalibrateOnly, batteryNominalVoltage);
        }   //toString

    }   //class MotorParams

    private TrcMotor actuatorMotor;
    private TrcPidActuator pidActuator;

    /**
     * This method contains the common init code that will be called by all constructor.
     *
     * @param instanceName specifies the instance name.
     * @param actuatorMotor specifies the actuator motor.
     * @param lowerLimitSw specifies the lower limit switch object.
     * @param upperLimitSw specifies the upper limit switch object.
     * @param motorParams specifies the parameters to set up the actuator motor.
     * @param actuatorParams specifies the parameters to set up the PID actuator.
     */
    private void commonInit(
        String instanceName, TrcMotor actuatorMotor, TrcDigitalInput lowerLimitSw, TrcDigitalInput upperLimitSw,
        MotorParams motorParams, TrcPidActuator.Parameters actuatorParams)
    {
        this.actuatorMotor = actuatorMotor;
        actuatorMotor.setBrakeModeEnabled(true);
        actuatorMotor.setInverted(motorParams.motorInverted);

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
     * @param motorParams specifies the parameters to set up the actuator motor.
     * @param actuatorParams specifies the parameters to set up the PID actuator.
     */
    public FrcMotorActuator(
        String instanceName, TrcMotor actuatorMotor, TrcDigitalInput lowerLimitSw, TrcDigitalInput upperLimitSw,
        MotorParams motorParams, TrcPidActuator.Parameters actuatorParams)
    {
        commonInit(instanceName, actuatorMotor, lowerLimitSw, upperLimitSw, motorParams, actuatorParams);
    }   //FrcMotorActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param actuatorMotor specifies the actuator motor.
     * @param motorParams specifies the parameters to set up the actuator motor.
     * @param actuatorParams specifies the parameters to set up the PID actuator.
     */
    public FrcMotorActuator(
        String instanceName, TrcMotor actuatorMotor, MotorParams motorParams, TrcPidActuator.Parameters actuatorParams)
    {
        FrcDigitalInput lowerLimitSwitch, upperLimitSwitch;

        if (motorParams.lowerLimitSwitchDigitalChannel >= 0)
        {
            lowerLimitSwitch = new FrcDigitalInput(instanceName + ".lowerLimitSw", motorParams.lowerLimitSwitchDigitalChannel);
            lowerLimitSwitch.setInverted(motorParams.lowerLimitSwitchInverted);
        }
        else
        {
            lowerLimitSwitch = null;
        }

        if (motorParams.upperLimitSwitchDigitalChannel >= 0)
        {
            upperLimitSwitch = new FrcDigitalInput(instanceName + ".upperLimitSw", motorParams.upperLimitSwitchDigitalChannel);
            upperLimitSwitch.setInverted(motorParams.upperLimitSwitchInverted);
        }
        else
        {
            upperLimitSwitch = null;
        }

        commonInit(instanceName, actuatorMotor, lowerLimitSwitch, upperLimitSwitch, motorParams, actuatorParams);
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
