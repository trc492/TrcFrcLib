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

import java.io.InputStream;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcLoopProfiler;
import TrcCommonLib.trclib.TrcPeriodicThread;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcRobot.*;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcWatchdogMgr;

/**
 * This class defines and implements the FrcRobotBase object. The FrcRobotBase object implements a cooperative
 * multitasking robot. Different subsystems register themselves as CoopTasks. FrcRobotBase uses the TaskMgr to
 * task switch between different subsystem tasks at various points in the robot loop. This basically simulates
 * a cooperative multitasking scheduler that task switches between them in different modes.
 */
public abstract class FrcRobotBase extends RobotBase
{
    private static final String moduleName = FrcRobotBase.class.getSimpleName();
    private static final boolean debugLoopTimeEnabled = false;
    private static final boolean dashboardEnabled = true;

    private boolean liveWindowEnabled = false;

    /**
     * This method is called to initialize the robot.
     */
    public abstract void robotInit();

    /**
     * This method is called to prepare the robot before a robot mode is about to start.
     *
     * @param runMode specifies the current run mode.
     * @param prevMode specifies the previous run mode.
     */
    public abstract void robotStartMode(RunMode runMode, RunMode prevMode);

    /**
     * This method is called to prepare the robot right after a robot mode has been stopped.
     *
     * @param runMode specifies the current run mode.
     * @param nextMode specifies the next run mode.
     */
    public abstract void robotStopMode(RunMode runMode, RunMode nextMode);

    /**
     * This method is called periodically in the specified run mode. This is typically used to execute periodic tasks
     * that's common to all run modes.
     *
     * @param runMode specifies the current run mode.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    public abstract void robotPeriodic(RunMode runMode, boolean slowPeriodicLoop);

    private final TrcDbgTrace globalTracer;
    private final FrcDashboard dashboard;
    private final String robotName;
    private static FrcRobotBase instance;
    private Thread robotThread;
    private TrcWatchdogMgr.Watchdog robotThreadWatchdog;
    private volatile boolean terminate = false;

    private TrcLoopProfiler robotMainLoopProfiler = new TrcLoopProfiler(moduleName);
    private static long loopCounter = 0;
    private static long slowLoopCounter = 0;
    private double nextSlowLoopTime = 0.0;
    private RunMode prevMode = RunMode.INVALID_MODE;
    private RunMode currMode = RunMode.INVALID_MODE;

    private RobotMode teleOpMode = null;
    private RobotMode autoMode = null;
    private RobotMode testMode = null;
    private RobotMode disabledMode = null;
    private TrcEvent sysActiveEvent = null;
    private TrcEvent.Callback sysActiveEventCallback = null;
    private boolean isSysActive = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robotName specifies the robot name.
     */
    public FrcRobotBase(String robotName)
    {
        super();

        if (FrcRobotBase.instance != null)
        {
            throw new RuntimeException("FrcRobotBase has already been instantiated.");
        }
        //
        // Must be done before instantiating TrcDbgTrace.
        //
        this.globalTracer = new TrcDbgTrace(moduleName, new FrcDbgLog());
        this.dashboard = FrcDashboard.getInstance();
        this.robotName = robotName;
        FrcRobotBase.instance = this;
        dashboard.clearDisplay();
    }   //FrcRobotBase

    /**
     * This method returns the saved instance. This is a static method. So other class can get to this class instance
     * by calling getInstance(). This is very useful for other classes that need to access the public fields and
     * methods.
     *
     * @return save instance of this class.
     */
    public static FrcRobotBase getInstance()
    {
        return instance;
    }   //getInstance

    /**
     * This method returns the robot name.
     *
     * @return robot name.
     */
    @Override
    public String toString()
    {
        return robotName;
    }   //toString

    /**
     * This method returns the dashboard object.
     *
     * @return dashboard object.
     */
    public FrcDashboard getDashboard()
    {
        return dashboard;
    }   //getDashboard

    /**
     * This method returns the loop counter. This is very useful for code to determine if it is called
     * multiple times in the same loop. For example, it can be used to optimize sensor access so that if
     * the sensor is accessed in the same loop, there is no reason to create a new bus transaction to get "fresh"
     * data from the sensor.
     *
     * @return loop counter value.
     */
    public static long getLoopCounter()
    {
        return loopCounter;
    }   //getLoopCounter

    /**
     * This method returns the slow loop counter. This is very useful for code to determine if it is called
     * multiple times in the same slow loop. For example, it can be used to optimize sensor access so that if
     * the sensor is accessed in the same loop, there is no reason to create a new bus transaction to get "fresh"
     * data from the sensor.
     *
     * @return slow loop counter value.
     */
    public static long getSlowLoopCounter()
    {
        return slowLoopCounter;
    }   //getSlowLoopCounter

    /**
     * This method returns the current run mode.
     *
     * @return current run mode.
     */
    public RunMode getCurrentRunMode()
    {
        return currMode;
    }   //getCurrentRunMode

    /**
     * This method is called by the subclass to set up various robot mode objects.
     *
     * @param teleOpMode specifies the TeleOp mode object.
     * @param autoMode specifies the Autonomous mode object.
     * @param testMode specifies the Test mode object.
     * @param disabledMode specifies the Disabled mode object.
     */
    public void setupRobotModes(RobotMode teleOpMode, RobotMode autoMode, RobotMode testMode, RobotMode disabledMode)
    {
        this.teleOpMode = teleOpMode;
        this.autoMode = autoMode;
        this.testMode = testMode;
        this.disabledMode = disabledMode;
    }   //setupRobotModes

    /**
     * This method returns the TeleOp robot mode object.
     *
     * @return TeleOp robot mode object or null if not yet set.
     */
    public RobotMode getTeleOpMode()
    {
        return teleOpMode;
    }   //getTeleOpMode

    /**
     * This method returns the Autonomous robot mode object.
     *
     * @return Autonomous robot mode object or null if not yet set.
     */
    public RobotMode getAutoMode()
    {
        return autoMode;
    }   //getAutoMode

    /**
     * This method returns the Test robot mode object.
     *
     * @return Test robot mode object or null if not yet set.
     */
    public RobotMode getTestMode()
    {
        return testMode;
    }   //getTestMode

    /**
     * This method returns the Disabled robot mode object.
     *
     * @return Disabled robot mode object or null if not yet set.
     */
    public RobotMode getDisabledMode()
    {
        return disabledMode;
    }   //getDisabledMode

    /**
     * This method sends a heart beat to the main robot thread watchdog. This is important if during robot init time
     * that the user code decided to synchronously busy wait for something, it must periodically call this method to
     * prevent the watchdog from complaining.
     */
    public void sendWatchdogHeartBeat()
    {
        if (Thread.currentThread() == robotThread)
        {
            if (robotThreadWatchdog != null)
            {
                TrcEvent.performEventCallback();
                robotThreadWatchdog.sendHeartBeat();
            }
            else
            {
                globalTracer.traceWarn(moduleName, "Robot thread watchdog has not been created yet.");
                TrcDbgTrace.printThreadStack();
            }
        }
        else
        {
            globalTracer.traceWarn(moduleName, "Caller must be on the OpMode thread to call this.");
            TrcDbgTrace.printThreadStack();
        }
    }   //sendWatchdogHeartBeat

    /**
     * This method enables/disables SysActive monitoring. This is very useful for detecting comm lost or comm
     * reconnect.
     *
     * @param callback specifies the callback handler to enable SysActive monitoring, null to disable.
     */
    public void setSysActiveMonitorEnabled(TrcEvent.Callback callback)
    {
        if (callback != null)
        {
            sysActiveEvent = new TrcEvent("SysActive");
            isSysActive = RobotController.isSysActive();
        }
        else
        {
            sysActiveEvent = null;
        }
        this.sysActiveEventCallback = callback;
    }   //setSysActiveMonitorEnabled

    /**
     * This method returns the sysActiveEvent object if sysActive monitor is enabled..
     *
     * @return sysActiveEvent object, null if sysActive Monitor is not enabled.
     */
    public TrcEvent getSysActiveEvent()
    {
        return sysActiveEvent;
    }   //getSysActiveEvent

    /**
     * Start the competition match. This specific startCompetition() implements "main loop" behavior like that of the
     * FRC TimedRobot, with a primary "fast loop" running at a fast periodic rate (default 50 Hz). In addition, it
     * also runs a "slow loop" running at a slow periodic rate (default to 20 Hz). This code needs to track the order
     * of the modes starting to ensure that everything happens in the right order. Repeatedly run the correct method,
     * either Autonomous or TeleOp when the robot is enabled. After running the correct method, wait for some state
     * to change, either the other mode starts or the robot is disabled. Then go back and wait for the robot to be
     * enabled again.
     */
    public void startCompetition()
    {
        final double slowPeriodicInterval = 0.05;   // 50 msec (20 Hz).
        double periodicInterval = TrcTaskMgr.PERIODIC_INTERVAL_MS/1000.0;
        long startNanoTime;

        globalTracer.traceInfo(
            moduleName,
            "\n****************************************\n" +
            " Host Name: " + getHostName() + "\n" +
            " Robot Name: " + robotName + "\n"+
            "\n****************************************\n");

        robotThread = Thread.currentThread();
        robotThreadWatchdog = TrcWatchdogMgr.registerWatchdog(Thread.currentThread().getName() + ".watchdog");
        TrcEvent.registerEventCallback();
        // Running robotInit.
        globalTracer.traceDebug(moduleName, "Running robotInit.");
        startNanoTime = TrcTimer.getNanoTime();
        robotInit();
        robotMainLoopProfiler.recordProfilePointElapsedTime("RobotInit", startNanoTime, false);
        TrcPeriodicThread.setRobotInitialized(true);
        //
        // WPILib house keeping.
        //
        DriverStationModeThread modeThread = new DriverStationModeThread();
        int event = WPIUtilJNI.createEvent(false, false);
        DriverStation.provideRefreshedDataEventHandle(event);
        // Tell the DS that the robot is ready to be enabled.
        DriverStationJNI.observeUserProgramStarting();
        liveWindowEnabled = false;
        LiveWindow.setEnabled(liveWindowEnabled);
        //
        // Loop forever, calling the appropriate mode-dependent functions.
        //
        while (!Thread.currentThread().isInterrupted() && !terminate)
        {
            robotMainLoopProfiler.recordLoopStartTime();
            //
            // Determine the current run mode.
            //
            prevMode = currMode;
            if (isDisabled())
            {
                currMode = RunMode.DISABLED_MODE;
            }
            else if (isTest())
            {
                currMode = RunMode.TEST_MODE;
            }
            else if (isAutonomous())
            {
                currMode = RunMode.AUTO_MODE;
            }
            else if (isTeleop())
            {
                currMode = RunMode.TELEOP_MODE;
            }
            else
            {
                currMode = RunMode.INVALID_MODE;
            }

            if (currMode != prevMode)
            {
                //
                // Detected mode transition.
                //
                globalTracer.traceInfo(moduleName, "*** Transitioning from " + prevMode + " to " + currMode + " ***");

                if (prevMode != RunMode.INVALID_MODE)
                {
                    //
                    // Execute all stop tasks for previous mode.
                    //
                    globalTracer.traceDebug(moduleName, "Running " + prevMode + ".stopTask.");
                    startNanoTime = TrcTimer.getNanoTime();
                    TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, prevMode, false);
                    robotMainLoopProfiler.recordProfilePointElapsedTime("StopTask", startNanoTime, true);
                    //
                    // Stop previous mode.
                    //
                    globalTracer.traceDebug(moduleName, "Running " + prevMode + ".stopMode.");
                    startNanoTime = TrcTimer.getNanoTime();
                    if (prevMode == RunMode.DISABLED_MODE && disabledMode != null)
                    {
                        disabledMode.stopMode(prevMode, currMode);
                    }
                    else if (prevMode == RunMode.TEST_MODE && testMode != null)
                    {
                        testMode.stopMode(prevMode, currMode);
                    }
                    else if (prevMode == RunMode.AUTO_MODE && autoMode != null)
                    {
                        autoMode.stopMode(prevMode, currMode);
                    }
                    else if (prevMode == RunMode.TELEOP_MODE && teleOpMode != null)
                    {
                        teleOpMode.stopMode(prevMode, currMode);
                    }
                    robotMainLoopProfiler.recordProfilePointElapsedTime("StopMode", startNanoTime, true);
                    //
                    // Run robotStopMode for the previous mode.
                    //
                    globalTracer.traceDebug(moduleName, "Running " + prevMode + ".robotStopMode.");
                    startNanoTime = TrcTimer.getNanoTime();
                    robotStopMode(prevMode, currMode);
                    robotMainLoopProfiler.recordProfilePointElapsedTime("RobotStopMode", startNanoTime, true);

                    if (debugLoopTimeEnabled)
                    {
                        TrcTaskMgr.printAllRegisteredTasks();
                    }
                }

                TrcRobot.setRunMode(currMode);
                TrcTimer.recordModeStartTime();
                if (currMode != RunMode.INVALID_MODE)
                {
                    //
                    // Run robotStartMode for the current mode.
                    //
                    globalTracer.traceDebug(moduleName, "Running " + currMode + ".robotStartMode.");
                    startNanoTime = TrcTimer.getNanoTime();
                    robotStartMode(currMode, prevMode);
                    robotMainLoopProfiler.recordProfilePointElapsedTime("RobotStartMode", startNanoTime, true);
                    //
                    // Start current mode.
                    //
                    globalTracer.traceDebug(moduleName, "Running " + currMode + ".startMode.");
                    startNanoTime = TrcTimer.getNanoTime();
                    if (currMode == RunMode.DISABLED_MODE)
                    {
                        liveWindowEnabled = false;
                        if (disabledMode != null)
                        {
                            disabledMode.startMode(prevMode, currMode);
                        }
                    }
                    else if (currMode == RunMode.TEST_MODE)
                    {
                        liveWindowEnabled = true;
                        if (testMode != null)
                        {
                            testMode.startMode(prevMode, currMode);
                        }
                    }
                    else if (currMode == RunMode.AUTO_MODE)
                    {
                        liveWindowEnabled = false;
                        if (autoMode != null)
                        {
                            autoMode.startMode(prevMode, currMode);
                        }
                    }
                    else if (currMode == RunMode.TELEOP_MODE)
                    {
                        liveWindowEnabled = false;
                        if (teleOpMode != null)
                        {
                            teleOpMode.startMode(prevMode, currMode);
                        }
                    }
                    LiveWindow.setEnabled(liveWindowEnabled);
                    robotMainLoopProfiler.recordProfilePointElapsedTime("StartMode", startNanoTime, true);
                    //
                    // Execute all start tasks for current mode.
                    //
                    globalTracer.traceDebug(moduleName, "Running " + ".startTask.");
                    startNanoTime = TrcTimer.getNanoTime();
                    TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, currMode, false);
                    robotMainLoopProfiler.recordProfilePointElapsedTime("StartTask", startNanoTime, true);
                }
                //
                // Reset all performance counters for the mode.
                //
                loopCounter = 0;
                slowLoopCounter = 0;
                nextSlowLoopTime = robotMainLoopProfiler.getLoopStartTime();
            }

            //
            // Run the time slice.
            //
            double modeElapsedTime = TrcTimer.getModeElapsedTime();
            double currTime = robotMainLoopProfiler.getHighPrecisionCurrentTime();
            boolean slowPeriodicLoop = currTime >= nextSlowLoopTime;

            loopCounter++;
            if (slowPeriodicLoop)
            {
                nextSlowLoopTime = currTime + slowPeriodicInterval;
                slowLoopCounter++;
            }
            // SysActive monitor is enabled.
            if (sysActiveEventCallback != null)
            {
                boolean sysActive = RobotController.isSysActive();

                if (isSysActive ^ sysActive)
                {
                    globalTracer.traceInfo(
                        moduleName,
                        "***** SysActive: robot is " + (sysActive? "connected": "disconnected") + ". *****");
                    sysActiveEvent.setCallback(sysActiveEventCallback, sysActive);
                    sysActiveEvent.signal();
                    isSysActive = sysActive;
                }
            }
            //
            // PrePeriodic.
            //
            globalTracer.traceDebug(moduleName, "Running " + currMode + ".prePeriodicTask.");
            startNanoTime = TrcTimer.getNanoTime();
            TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.PRE_PERIODIC_TASK, currMode, slowPeriodicLoop);
            robotMainLoopProfiler.recordProfilePointElapsedTime("PrePeriodicTask", startNanoTime, true);
            //
            // Perform event callback here because pre-periodic tasks have finished processing sensor inputs and
            // may have signaled events. We will do all the callbacks before running periodic code.
            //
            TrcEvent.performEventCallback();
            //
            // Periodic.
            //
            globalTracer.traceDebug(moduleName, "Running " + currMode + ".periodic.");
            startNanoTime = TrcTimer.getNanoTime();
            if (currMode == RunMode.DISABLED_MODE && disabledMode != null)
            {
                modeThread.inDisabled(true);
                robotPeriodic(currMode, slowPeriodicLoop);
                disabledMode.periodic(modeElapsedTime, slowPeriodicLoop);
                modeThread.inDisabled(false);
            }
            else if (currMode == RunMode.TEST_MODE && testMode != null)
            {
                modeThread.inTest(true);
                robotPeriodic(currMode, slowPeriodicLoop);
                testMode.periodic(modeElapsedTime, slowPeriodicLoop);
                modeThread.inTest(false);
            }
            else if (currMode == RunMode.AUTO_MODE && autoMode != null)
            {
                modeThread.inAutonomous(true);
                robotPeriodic(currMode, slowPeriodicLoop);
                autoMode.periodic(modeElapsedTime, slowPeriodicLoop);
                modeThread.inAutonomous(false);
            }
            else if (currMode == RunMode.TELEOP_MODE && teleOpMode != null)
            {
                modeThread.inTeleop(true);
                robotPeriodic(currMode, slowPeriodicLoop);
                teleOpMode.periodic(modeElapsedTime, slowPeriodicLoop);
                modeThread.inTeleop(false);
            }
            robotMainLoopProfiler.recordProfilePointElapsedTime("Periodic", startNanoTime, true);
            //
            // PostPeriodic.
            //
            globalTracer.traceDebug(moduleName, "Running " + currMode + ".postPeriodicTasks.");
            startNanoTime = TrcTimer.getNanoTime();
            TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.POST_PERIODIC_TASK, currMode, slowPeriodicLoop);
            robotMainLoopProfiler.recordProfilePointElapsedTime("PostPeriodicTask", startNanoTime, true);

            startNanoTime = TrcTimer.getNanoTime();
            SmartDashboard.updateValues();
            if (liveWindowEnabled)
            {
                LiveWindow.updateValues();
            }

            if (dashboardEnabled && slowPeriodicLoop)
            {
                //
                // Only update dashboard running time at periodic rate.
                //
                dashboard.displayPrintf(0, "[%3d:%06.3f] %s", (int)(modeElapsedTime/60), modeElapsedTime%60, currMode);
            }
            robotMainLoopProfiler.recordProfilePointElapsedTime("UpdateTask", startNanoTime, true);

            robotThreadWatchdog.sendHeartBeat();
            //
            // Do house keeping statistics and keep loop timeslice timing.
            // If periodicInterval (timeslice) is not zero and we haven't used up the timeslice, we will sleep the
            // rest of the timeslice.
            //
            if (periodicInterval > 0.0)
            {
                double loopTime = robotMainLoopProfiler.getHighPrecisionCurrentTime() - robotMainLoopProfiler.getLoopStartTime();
                if (loopTime >= periodicInterval*2.0)
                {
                    globalTracer.traceWarn(
                        moduleName, currMode + " took too long (" + loopTime + "s/" + periodicInterval + "s)");
                }
                else
                {
                    TrcTimer.sleep((long) ((periodicInterval - loopTime)*1000));
                }
            }
        }

        DriverStation.removeRefreshedDataEventHandle(event);
        modeThread.close();
    }   //startCompetition

    /**
     * Called to end the competition loop.
     */
    @Override
    public void endCompetition()
    {
        TrcEvent.unregisterEventCallback();
        robotThreadWatchdog.unregister();
        robotThreadWatchdog = null;
        terminate = true;
    }   //endCompetition

    /**
     * This method prints the performance metrics of all loops and taska.
     *
     * @param tracer specifies the tracer to be used for printing the performance metrics.
     */
    public void printPerformanceMetrics(TrcDbgTrace tracer)
    {
        robotMainLoopProfiler.printPerformanceMetrics(tracer);
        TrcTaskMgr.printTaskPerformanceMetrics();
    }   //printPerformanceMetrics

    /**
     * This method returns the host name of the RobotRIO.
     *
     * @return host name.
     */
    private String getHostName()
    {
        String hostName = null;

        try
        {
            byte[] buff = new byte[256];
            Process proc = Runtime.getRuntime().exec("hostname");
            InputStream inStream = proc.getInputStream();
            inStream.read(buff, 0, buff.length);
            hostName = new String(buff);
        }
        catch(IOException e)
        {
            e.printStackTrace();
        }

        return hostName;
    }   //getHostName

}   //class FrcRobotBase
