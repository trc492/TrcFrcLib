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

/**
 * This class implements a platform dependent digital input sensor extending TrcDigitalInput. It provides
 * implementation of the abstract methods in TrcDigitalInput. The digital input sensor in this case is one
 * of the SparkMax limit switches. This allows the SparkMax limit switch to be used as a Digital Trigger
 * for operations such as auto zero calibration and limit switch notification callback.
 */
public class FrcCANSparkMaxLimitSwitch extends TrcDigitalInput
{
    private final FrcCANSparkMax canSparkMax;
    private final boolean upperLimitSwitch;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param canSparkMax specifies the SparkMax motor controller that hosted the limit switch.
     * @param upperLimitSwitch specifies true for the upper limit switch, false for lower limit switch.
     */
    public FrcCANSparkMaxLimitSwitch(String instanceName, FrcCANSparkMax canSparkMax, boolean upperLimitSwitch)
    {
        super(instanceName);
        this.canSparkMax = canSparkMax;
        this.upperLimitSwitch = upperLimitSwitch;
    }   //FrcCANSparkMaxLimitSwitch

    //
    // Implements TrcDigitalInput abstract methods.
    //

    /**
     * This method returns the state of the digital input sensor.
     *
     * @return true if the digital input sensor is active, false otherwise.
     */
    @Override
    public boolean getInputState()
    {
        return upperLimitSwitch? canSparkMax.isUpperLimitSwitchActive(): canSparkMax.isLowerLimitSwitchActive();
    }   //getInputState

    /**
     * This method inverts the polarity of the limit switch by configuring it to be normally open (non-inverted) or
     * normally close (inverted).
     *
     * @param inverted specifies true to invert and false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        if (upperLimitSwitch)
        {
            canSparkMax.setFwdLimitSwitchInverted(inverted);
        }
        else
        {
            canSparkMax.setRevLimitSwitchInverted(inverted);
        }
    }   //setInverted

}   //class FrcCANSparkMaxLimitSwitch
