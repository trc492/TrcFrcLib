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
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
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
    private static final String moduleName = "FrcRobotBase";
    private static final boolean debugEnabled = false;

    protected boolean liveWindowEnabled = false;
    protected boolean debugPerformanceEnabled = false;
    private static final boolean debugLoopTimeEnabled = false;

    private static final boolean dashboardEnabled = true;

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

    private final TrcDbgTrace globalTracer;
    private final FrcDashboard dashboard;
    private final String robotName;
    private static FrcRobotBase instance;
    private Thread robotThread;
    private TrcWatchdogMgr.Watchdog robotThreadWatchdog;
    private volatile boolean terminate = false;

    private Double prevLoopStartTime = null;
    private double robotInitElapsedTime = 0.0;
    private double stopTaskElapsedTime = 0.0;
    private double stopModeElapsedTime = 0.0;
    private double robotStopModeElapsedTime = 0.0;
    private double robotStartModeElapsedTime = 0.0;
    private double startModeElapsedTime = 0.0;
    private double startTaskElapsedTime = 0.0;
    private static long loopCounter = 0;
    private static long slowLoopCounter = 0;
    private double prePeriodicTaskTotalElapsedTime = 0.0;
    private double prePeriodicTaskMaxElapsedTime = 0.0;
    private double periodicTotalElapsedTime = 0.0;
    private double periodicMaxElapsedTime = 0.0;
    private double postPeriodicTaskTotalElapsedTime = 0.0;
    private double postPeriodicTaskMaxElapsedTime = 0.0;
    private double updatesTotalElapsedTime = 0.0;
    private double updatesMaxElapsedTime = 0.0;
    private double nextSlowLoopTime = 0.0;
    private RunMode prevMode = RunMode.INVALID_MODE;
    private RunMode currMode = RunMode.INVALID_MODE;

    private RobotMode teleOpMode = null;
    private RobotMode autoMode = null;
    private RobotMode testMode = null;
    private RobotMode disabledMode = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robotName specifies the robot name.
     */
    public FrcRobotBase(final String robotName)
    {
        super();

        if (FrcRobotBase.instance != null)
        {
            throw new RuntimeException("FrcRobotBase has already been instantiated.");
        }
        //
        // Must be done before instantiating TrcDbgTrace.
        //
        TrcDbgTrace.setDbgLog(new FrcDbgLog());
        globalTracer = TrcDbgTrace.getGlobalTracer();
        dashboard = FrcDashboard.getInstance();

        this.robotName = robotName;
        FrcRobotBase.instance = this;
        // Initialize modeStartTime just in case somebody's calling TrcUtil.getModeElapsedTime before it's initialized.
        TrcTimer.recordModeStartTime();
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
        final String funcName = "sendWatchdogHeartBeat";

        if (Thread.currentThread() == robotThread)
        {
            if (robotThreadWatchdog != null)
            {
                TrcEvent.performEventCallback();
                robotThreadWatchdog.sendHeartBeat();
            }
            else
            {
                globalTracer.traceWarn(funcName, "Robot thread watchdog has not been created yet.");
                TrcDbgTrace.printThreadStack();
            }
        }
        else
        {
            globalTracer.traceWarn(funcName, "Caller must be on the OpMode thread to call this.");
            TrcDbgTrace.printThreadStack();
        }
    }   //sendWatchdogHeartBeat

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
        final String funcName = "startCompetition";
        final double periodicInterval = TrcTaskMgr.PERIODIC_INTERVAL_MS/1000.0;
        final double slowPeriodicInterval = 0.05;   // 50 msec (20 Hz).
        final double taskTimeThreshold = TrcTaskMgr.TASKTIME_THRESHOLD_MS/1000.0;
        String bannerPrefix, bannerSuffix;
        double startTime, elapsedTime;

        if (FrcDbgLog.useEscSeq)
        {
            bannerPrefix = FrcDbgLog.ESC_PREFIX + FrcDbgLog.SGR_FG_BLACK +
                           FrcDbgLog.ESC_SEP + FrcDbgLog.SGR_BG_WHITE +
                           FrcDbgLog.ESC_SUFFIX;
            bannerSuffix = FrcDbgLog.ESC_NORMAL;
        }
        else
        {
            bannerPrefix = "";
            bannerSuffix = "";
        }

        globalTracer.tracePrintf(
            bannerPrefix +
            "\n****************************************\n" +
            " Host Name: %s\n" +
            "Robot Name: %s\n"+
            "\n****************************************\n" +
            bannerSuffix,
            getHostName(), robotName);

        robotThread = Thread.currentThread();
        robotThreadWatchdog = TrcWatchdogMgr.registerWatchdog(Thread.currentThread().getName() + ".watchdog");
        TrcEvent.registerEventCallback();

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "[%.3f] running robotInit.", TrcTimer.getModeElapsedTime());
        }
        startTime = TrcTimer.getCurrentTime();
        robotInit();
        robotInitElapsedTime = TrcTimer.getCurrentTime() - startTime;
        if (debugPerformanceEnabled)
        {
            globalTracer.traceInfo(
                funcName, "[%.3f] robotInitElapsedTime=%.6fs", TrcTimer.getModeElapsedTime(), robotInitElapsedTime);
        }
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
            double loopStartTime = TrcTimer.getCurrentTime();

            if (debugLoopTimeEnabled)
            {
                if (prevLoopStartTime != null)
                {
                    globalTracer.traceInfo(funcName, "Loop Interval = %.6fs\n", loopStartTime - prevLoopStartTime);
                }
            }
            prevLoopStartTime = loopStartTime;
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
                globalTracer.traceInfo(funcName, "*** Transitioning from %s to %s ***", prevMode, currMode);
                TrcTimer.recordModeStartTime();

                if (prevMode != RunMode.INVALID_MODE)
                {
                    //
                    // Execute all stop tasks for previous mode.
                    //
                    if (debugEnabled)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] running %s.stopTasks.", TrcTimer.getModeElapsedTime(), prevMode);
                    }
                    startTime = TrcTimer.getCurrentTime();
                    TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, prevMode, false);
                    stopTaskElapsedTime = TrcTimer.getCurrentTime() - startTime;
                    if (debugPerformanceEnabled)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] %s.stopTaskElapsedTime=%.6fs",
                            TrcTimer.getModeElapsedTime(), prevMode, stopTaskElapsedTime);
                    }
                    //
                    // Stop previous mode.
                    //
                    if (debugEnabled)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] running %s.stopMode.",TrcTimer.getModeElapsedTime(), prevMode);
                    }
                    startTime = TrcTimer.getCurrentTime();
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
                    stopModeElapsedTime = TrcTimer.getCurrentTime() - startTime;
                    if (debugPerformanceEnabled)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] %s.stopModeElapsedTime=%.6fs",
                            TrcTimer.getModeElapsedTime(), prevMode, stopModeElapsedTime);
                    }
                    //
                    // Run robotStopMode for the previous mode.
                    //
                    if (debugEnabled)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] running %s.robotStopMode.", TrcTimer.getModeElapsedTime(), prevMode);
                    }
                    startTime = TrcTimer.getCurrentTime();
                    robotStopMode(prevMode, currMode);
                    robotStopModeElapsedTime = TrcTimer.getCurrentTime() - startTime;
                    if (debugPerformanceEnabled)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] %s.robotStopModeElapsedTime=%.6fs",
                            TrcTimer.getModeElapsedTime(), prevMode, robotStopModeElapsedTime);
                    }

                    if (debugLoopTimeEnabled)
                    {
                        TrcTaskMgr.printAllRegisteredTasks(globalTracer);
                    }
                }

                TrcRobot.setRunMode(currMode);
                if (currMode != RunMode.INVALID_MODE)
                {
                    //
                    // Run robotStartMode for the current mode.
                    //
                    if (debugEnabled)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] running %s.robotStartMode.", TrcTimer.getModeElapsedTime(), currMode);
                    }
                    startTime = TrcTimer.getCurrentTime();
                    robotStartMode(currMode, prevMode);
                    robotStartModeElapsedTime = TrcTimer.getCurrentTime() - startTime;
                    if (debugPerformanceEnabled)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] %s.robotStartModeElapsedTime=%.6fs",
                            TrcTimer.getModeElapsedTime(), currMode, robotStartModeElapsedTime);
                    }
                    //
                    // Start current mode.
                    //
                    if (debugEnabled)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] running %s.startMode.", TrcTimer.getModeElapsedTime(), currMode);
                    }
                    startTime = TrcTimer.getCurrentTime();
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
                    startModeElapsedTime = TrcTimer.getCurrentTime() - startTime;
                    if (debugPerformanceEnabled)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] %s.startModeElapsedTime=%.6fs",
                            TrcTimer.getModeElapsedTime(), currMode, startModeElapsedTime);
                    }
                    //
                    // Execute all start tasks for current mode.
                    //
                    if (debugEnabled)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] running %s.startTasks.", TrcTimer.getModeElapsedTime(), currMode);
                    }
                    startTime = TrcTimer.getCurrentTime();
                    TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, currMode, false);
                    startTaskElapsedTime = TrcTimer.getCurrentTime() - startTime;
                    if (debugPerformanceEnabled)
                    {
                        globalTracer.traceInfo(
                            funcName, "[%.3f] %s.startTaskElapsedTime=%.6fs",
                            TrcTimer.getModeElapsedTime(), currMode, startTaskElapsedTime);
                    }
                }
                //
                // Reset all performance counters for the mode.
                //
                loopCounter = 0;
                slowLoopCounter = 0;
                prePeriodicTaskTotalElapsedTime = 0.0;
                prePeriodicTaskMaxElapsedTime = 0.0;
                periodicTotalElapsedTime = 0.0;
                periodicMaxElapsedTime = 0.0;
                postPeriodicTaskTotalElapsedTime = 0.0;
                postPeriodicTaskMaxElapsedTime = 0.0;
                updatesTotalElapsedTime = 0.0;
                updatesMaxElapsedTime = 0.0;
                nextSlowLoopTime = loopStartTime;
            }

            //
            // Run the time slice.
            //
            double modeElapsedTime = TrcTimer.getModeElapsedTime();
            double currTime = TrcTimer.getCurrentTime();
            boolean slowPeriodicLoop = currTime >= nextSlowLoopTime;

            loopCounter++;
            if (slowPeriodicLoop)
            {
                nextSlowLoopTime = currTime + slowPeriodicInterval;
                slowLoopCounter++;
            }
            //
            // PrePeriodic.
            //
            if (debugEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "[%.3f] running %s.prePeriodicTasks.", TrcTimer.getModeElapsedTime(), currMode);
            }
            startTime = TrcTimer.getCurrentTime();
            TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.PRE_PERIODIC_TASK, currMode, slowPeriodicLoop);
            elapsedTime = TrcTimer.getCurrentTime() - startTime;
            prePeriodicTaskTotalElapsedTime += elapsedTime;
            if (elapsedTime > prePeriodicTaskMaxElapsedTime) prePeriodicTaskMaxElapsedTime = elapsedTime;
            if (debugPerformanceEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "[%.3f] %s.prePeriodicTaskElapsedTime=%.6f/%.6f/%.6fs",
                    TrcTimer.getModeElapsedTime(), currMode, elapsedTime, prePeriodicTaskTotalElapsedTime/loopCounter,
                    prePeriodicTaskMaxElapsedTime);
            }
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(
                    funcName, "%s.prePeriodicTasks took too long (%.3fs)", currMode, elapsedTime);
            }
            //
            // Perform event callback here because pre-periodic tasks have finished processing sensor inputs and
            // may have signaled events. We will do all the callbacks before running periodic code.
            //
            TrcEvent.performEventCallback();
            //
            // Periodic.
            //
            if (debugEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "[%.3f] running %s.periodic.", TrcTimer.getModeElapsedTime(), currMode);
            }
            startTime = TrcTimer.getCurrentTime();
            if (currMode == RunMode.DISABLED_MODE && disabledMode != null)
            {
                modeThread.inDisabled(true);
                disabledMode.periodic(modeElapsedTime, slowPeriodicLoop);
                modeThread.inDisabled(false);
            }
            else if (currMode == RunMode.TEST_MODE && testMode != null)
            {
                modeThread.inTest(true);
                testMode.periodic(modeElapsedTime, slowPeriodicLoop);
                modeThread.inTest(false);
            }
            else if (currMode == RunMode.AUTO_MODE && autoMode != null)
            {
                modeThread.inAutonomous(true);
                autoMode.periodic(modeElapsedTime, slowPeriodicLoop);
                modeThread.inAutonomous(false);
            }
            else if (currMode == RunMode.TELEOP_MODE && teleOpMode != null)
            {
                modeThread.inTeleop(true);
                teleOpMode.periodic(modeElapsedTime, slowPeriodicLoop);
                modeThread.inTeleop(false);
            }
            elapsedTime = TrcTimer.getCurrentTime() - startTime;
            periodicTotalElapsedTime += elapsedTime;
            if (elapsedTime > periodicMaxElapsedTime) periodicMaxElapsedTime = elapsedTime;
            if (debugPerformanceEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "[%.3f] %s.fastPeriodicElapsedTime=%.6f/%.6f/%.6fs",
                    TrcTimer.getModeElapsedTime(), currMode, elapsedTime, periodicTotalElapsedTime/loopCounter,
                    periodicMaxElapsedTime);
            }
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(
                    funcName, "%s.fastPeriodic took too long (%.3fs)", currMode, elapsedTime);
            }
            //
            // PostPeriodic.
            //
            if (debugEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "[%.3f] running %s.postPeriodicTasks.", TrcTimer.getModeElapsedTime(), currMode);
            }
            startTime = TrcTimer.getCurrentTime();
            TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.POST_PERIODIC_TASK, currMode, slowPeriodicLoop);
            elapsedTime = TrcTimer.getCurrentTime() - startTime;
            postPeriodicTaskTotalElapsedTime += elapsedTime;
            if (elapsedTime > postPeriodicTaskMaxElapsedTime) postPeriodicTaskMaxElapsedTime = elapsedTime;
            if (debugPerformanceEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "[%.3f] %s.fastPostPeriodicTaskElapsedTime=%.6f/%.6f/%.6fs",
                    TrcTimer.getModeElapsedTime(), currMode, elapsedTime, postPeriodicTaskTotalElapsedTime/loopCounter,
                    postPeriodicTaskMaxElapsedTime);
            }
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(
                    funcName, "%s.fastPostPeriodicTasks took too long (%.3fs)", currMode, elapsedTime);
            }

            startTime = TrcTimer.getCurrentTime();
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
            elapsedTime = TrcTimer.getCurrentTime() - startTime;
            updatesTotalElapsedTime += elapsedTime;
            if (elapsedTime > updatesMaxElapsedTime) updatesMaxElapsedTime = elapsedTime;
            if (debugPerformanceEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "[%.3f] %s.updatesElapsedTime=%.6f/%.6f/%.6fs",
                    TrcTimer.getModeElapsedTime(), currMode, elapsedTime, updatesTotalElapsedTime/loopCounter,
                    updatesTotalElapsedTime);
            }
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(
                    funcName, "%s.updates took too long (%.3fs)", currMode, elapsedTime);
            }

            robotThreadWatchdog.sendHeartBeat();
            //
            // Do house keeping statistics and keep loop timeslice timing.
            // If periodicInterval (timeslice) is not zero and we haven't used up the timeslice, we will sleep the
            // rest of the timeslice.
            //
            if (periodicInterval > 0.0)
            {
                double loopTime = TrcTimer.getCurrentTime() - loopStartTime;
                if (loopTime >= periodicInterval)
                {
                    globalTracer.traceWarn(
                        funcName, "%s took too long (%.3f/%.3f)", currMode, loopTime, periodicInterval);
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
        tracer.traceInfo(
            moduleName, "(%s->%s) Performance Metrics: LoopCount=%d", prevMode, currMode, loopCounter);
        tracer.traceInfo(
            moduleName,
            "RobotInit=%.6f, PrevModeStopTask=%.6f, PrevStopMode=%.6f, PrevRobotStopMode=%.6f, " +
            "RobotStartMode=%.6f, startMode=%.6f, startTask=%.6f",
            robotInitElapsedTime, stopTaskElapsedTime, stopModeElapsedTime, robotStopModeElapsedTime,
            robotStartModeElapsedTime, startModeElapsedTime, startTaskElapsedTime);
        tracer.traceInfo(
            moduleName, "PrePeriodicTask(Avg/Max)=%.6f/%.6f",
            prePeriodicTaskTotalElapsedTime/loopCounter, prePeriodicTaskMaxElapsedTime);
        tracer.traceInfo(
            moduleName, "Periodic(Avg/Max)=%.6f/%.6f", periodicTotalElapsedTime/loopCounter, periodicMaxElapsedTime);
        tracer.traceInfo(
            moduleName, "postPeriodicTask(Avg/Max)=%.6f/%.6f",
            postPeriodicTaskTotalElapsedTime/loopCounter, postPeriodicTaskMaxElapsedTime);
        tracer.traceInfo(
            moduleName, "Updates(Avg/Max)=%.6f/%.6f",
            updatesTotalElapsedTime/slowLoopCounter, updatesMaxElapsedTime);

        TrcTaskMgr.printTaskPerformanceMetrics(tracer);
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
