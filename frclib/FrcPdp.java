/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.Arrays;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.PowerDistribution;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;

/**
 * This class extends the WPI PowerDistricbutonPanel class to provide monitoring of energy consumption of registered
 * power channel.
 */
public class FrcPdp extends PowerDistribution
{
    private static final String moduleName = FrcPdp.class.getSimpleName();

    /**
     * This class contains the info of a PDP channel.
     */
    public static class Channel
    {
        public int channel;
        public String name;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param channel specifies the PDP channel number.
         * @param name specifies the name associated with the channel.
         */
        public Channel(int channel, String name)
        {
            this.channel = channel;
            this.name = name;
        }   //Channel

        /**
         * This method returns the string representation of the object.
         */
        public String toString()
        {
            return channel + ":" + name;
        }   //toString

    }   //class Channel

    private final int numChannels;
    private final String[] channelNames;
    private final double[] channelEnergyUsed;
    private final TrcTaskMgr.TaskObject energyUsedTaskObj;
    private double lastTimestamp = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param canId specifies the CAN ID of the PDP.
     * @param moduleType specifies the module type (automatic, CTRE, or REV).
     */
    public FrcPdp(int canId, ModuleType moduleType)
    {
        super(canId, moduleType);
        numChannels = getNumChannels();
        channelNames = new String[numChannels];
        channelEnergyUsed = new double[numChannels];
        energyUsedTaskObj = TrcTaskMgr.createTask(moduleName + ".energyUsedTask", this::energyUsedTask);

        Arrays.fill(channelNames, null);
        Arrays.fill(channelEnergyUsed, 0.0);
    }   //FrcPdp

    /**
     * This method enables/disables the energy monitoring task. When the task is enabled, it also clears the
     * totalEnergyUsed array.
     *
     * @param enabled specifies true to enable the task, false to disable.
     */
    public void setTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            synchronized (this)
            {
                for (int i = 0; i < numChannels; i++)
                {
                    channelEnergyUsed[i] = 0.0;
                }

                lastTimestamp = TrcTimer.getCurrentTime();
            }
            energyUsedTaskObj.registerTask(TrcTaskMgr.TaskType.STANDALONE_TASK);
        }
        else
        {
            energyUsedTaskObj.unregisterTask();
        }
    }   //setTaskEnabled

    /**
     * This method registers a PDP channel for monitoring its energy used.
     *
     * @param channel specifies the channel to be registered.
     * @param name specifies the channel name.
     * @return true if registered successfully, false if channel is invalid or already registered.
     */
    public boolean registerEnergyUsed(int channel, String name)
    {
        boolean success;

        synchronized (this)
        {
            success = channel >= 0 && channel < numChannels && channelNames[channel] == null;
            if (success)
            {
                channelNames[channel] = name;
                channelEnergyUsed[channel] = 0.0;
            }
        }

        return success;
    }   //registerEnergyUsed

    /**
     * This method registers a PDP channel for monitoring its energy used.
     *
     * @param channelInfo specifies the array of channels to be registered.
     * @return true if registered successfully, false if channel is invalid or already registered.
     */
    public boolean registerEnergyUsed(Channel... channels)
    {
        boolean success = true;

        synchronized (this)
        {
            for (Channel ch: channels)
            {
                success = registerEnergyUsed(ch.channel, ch.name);
                if (!success)
                {
                    break;
                }
            }
        }

        return success;
    }   //registerEnergyUsed

    /**
     * This method registers all currently unregistered PDP channels for monitoring its energy used
     * with a default name based on the channel number.
     */
    public void registerEnergyUsedForAllUnregisteredChannels()
    {
        synchronized (this)
        {
            for (int i = 0; i < numChannels; i++)
            {
                if (channelNames[i] == null)
                {
                    channelNames[i] = "Channel_" + i;
                    channelEnergyUsed[i] = 0.0;
                }
            }
        }
    }   //registerEnergyUsed

    /**
     * This method unregisters a PDP channel for monitoring its energy used.
     *
     * @param channel specifies the channel to be unregistered.
     * @return true if unregistered successfully, false if channel is not registered.
     */
    public boolean unregisterEnergyUsed(int channel)
    {
        boolean success;

        synchronized (this)
        {
            success = channelNames[channel] != null;
            if (success)
            {
                channelNames[channel] = null;
                channelEnergyUsed[channel] = 0.0;
            }
        }

        return success;
    }   //unregisterEnergyUsed

    /**
     * This method unregisters all PDP channel for monitoring its energy used.
     *
     * @return true if unregistered successfully, false if channel is not registered.
     */
    public void unregisterAllEnergyUsed()
    {
        synchronized (this)
        {
            for (int i = 0; i < numChannels; i++)
            {
                if (channelNames[i] != null)
                {
                    unregisterEnergyUsed(i);
                }
            }
        }
    }   //unregisterAllEnergyUsed

    /**
     * This method returns the energy consumed so far by the specified channel in the unit of Watt-Hour.
     *
     * @param channel specifies the PDP channel.
     * @return energy consumed by the channel in Watt-Hour if registered, null if not registered.
     */
    public double getEnergyUsed(int channel)
    {
        double energyUsed;

        synchronized (this)
        {
            energyUsed = channelNames[channel] != null? channelEnergyUsed[channel]: 0.0;
        }

        return energyUsed;
    }   //getEnergyUsed

    /**
     * This method returns the name of the registered PDP channel.
     *
     * @param channel specifies the PDP channel.
     * @return PDP channel name if registered, null if not registered.
     */
    public synchronized String getChannelName(int channel)
    {
        return channelNames[channel];
    }   //getChannelName

    /**
     * This method is called periodically to integrate the power consumption of each channel.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    public void energyUsedTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        double currTime = TrcTimer.getCurrentTime();
        double voltage = getVoltage();

        synchronized (this)
        {
            for (int i = 0; i < numChannels; i++)
            {
                if (channelNames[i] != null)
                {
                    channelEnergyUsed[i] += voltage*getCurrent(i)*(currTime - lastTimestamp)/3600.0;
                }
            }
            lastTimestamp = currTime;
        }
    }   //energyUsedTask

    public Sendable getPdpSendable()
    {
        PdpInfo pdpInfo = new PdpInfo();
        SendableRegistry.setName(pdpInfo, moduleName);
        return pdpInfo;
    }   //getPdpSendable

    private class PdpInfo implements Sendable
    {
        @Override
        public void initSendable(SendableBuilder builder)
        {
            builder.setSmartDashboardType("PowerDistributionPanel");
            for (int i = 0; i < numChannels; ++i) {
                final int chan = i;
                builder.addDoubleProperty("Chan" + i, () -> FrcPdp.this.getCurrent(chan), null);
            }
            builder.addDoubleProperty("Voltage", FrcPdp.this::getVoltage, null);
            builder.addDoubleProperty("TotalCurrent", FrcPdp.this::getTotalCurrent, null);
        }   //initSendable
    }   //class PdpInfo

}   //class FrcPdp
