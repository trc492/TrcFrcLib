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

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType; 
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcRobot.*;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcWatchdogMgr;

/**
 * This class defines and implements the FrcRobotBase object. The FrcRobotBase object implements a cooperative
 * multitasking robot. Different subsystems register themselves as CoopTasks. FrcRobotBase uses the TaskMgr to
 * task switch between different subsystem tasks at various points in the robot loop. This basically simulates
 * a cooperative multitasking scheduler that task switches between them in different modes.
 */
public abstract class FrcRobotBase extends RobotBase
{
    protected static final String moduleName = "FrcRobotBase";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;
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

    public TrcDbgTrace globalTracer;
    private FrcDashboard dashboard;
    private TrcWatchdogMgr.Watchdog mainThreadWatchdog;

    private static FrcRobotBase instance = null;

    private Double prevTimeSliceStartTime = null;
    private static long fastLoopCounter = 0;
    private static long slowLoopCounter = 0;

    private double nextSlowLoopTime = 0.0;
    private double robotInitElapsedTime = 0.0;
    private double stopTaskElapsedTime = 0.0;
    private double stopModeElapsedTime = 0.0;
    private double robotStopModeElapsedTime = 0.0;
    private double robotStartModeElapsedTime = 0.0;
    private double startModeElapsedTime = 0.0;
    private double startTaskElapsedTime = 0.0;
    private double fastPrePeriodicTaskTotalElapsedTime = 0.0;
    private double fastPrePeriodicTaskMaxElapsedTime = 0.0;
    private double slowPrePeriodicTaskTotalElapsedTime = 0.0;
    private double slowPrePeriodicTaskMaxElapsedTime = 0.0;
    private double fastPeriodicTotalElapsedTime = 0.0;
    private double fastPeriodicMaxElapsedTime = 0.0;
    private double slowPeriodicTotalElapsedTime = 0.0;
    private double slowPeriodicMaxElapsedTime = 0.0;
    private double fastPostPeriodicTaskTotalElapsedTime = 0.0;
    private double fastPostPeriodicTaskMaxElapsedTime = 0.0;
    private double slowPostPeriodicTaskTotalElapsedTime = 0.0;
    private double slowPostPeriodicTaskMaxElapsedTime = 0.0;
    private double updatesTotalElapsedTime = 0.0;
    private double updatesMaxElapsedTime = 0.0;

    private final String robotName;
    private RobotMode teleOpMode = null;
    private RobotMode autoMode = null;
    private RobotMode testMode = null;
    private RobotMode disabledMode = null;
    private RunMode prevMode = RunMode.INVALID_MODE;
    private RunMode currMode = RunMode.INVALID_MODE;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robotName specifies the robot name.
     */
    public FrcRobotBase(final String robotName)
    {
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
        TrcWatchdogMgr.getInstance();   // Create the Watchdog Manager singleton here.

        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                globalTracer:
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.robotName = robotName;
        FrcRobotBase.instance = this;
        // Initialize modeStartTime just in case somebody's calling TrcUtil.getModeElapsedTime before it's initialized.
        TrcUtil.recordModeStartTime();
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
     * This method returns the fast loop counter. This is very useful for code to determine if it is called
     * multiple times in the same fast loop. For example, it can be used to optimize sensor access so that if
     * the sensor is accessed in the same loop, there is no reason to create a new bus transaction to get "fresh"
     * data from the sensor.
     *
     * @return fast loop counter value.
     */
    public static long getFastLoopCounter()
    {
        return fastLoopCounter;
    }   //getFastLoopCounter

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
        final double fastLoopInterval = TrcTaskMgr.FAST_LOOP_INTERVAL_MS/1000.0;
        final double slowLoopInterval = TrcTaskMgr.SLOW_LOOP_INTERVAL_MS/1000.0;
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

        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Iterative);
        mainThreadWatchdog = TrcWatchdogMgr.registerWatchdog("MainThread");

        startTime = TrcUtil.getCurrentTime();
        robotInit();
        robotInitElapsedTime = TrcUtil.getCurrentTime() - startTime;
        if (debugPerformanceEnabled)
        {
            globalTracer.traceInfo(funcName, "robotInitElapsedTime=%.6fs", robotInitElapsedTime);
        }

        //
        // Tell the DS that the robot is ready to be enabled.
        //
        HAL.observeUserProgramStarting();

        liveWindowEnabled = false;
        LiveWindow.setEnabled(liveWindowEnabled);

        //
        // loop forever, calling the appropriate mode-dependent function
        //
        while (true)
        {
            double timeSliceStartTime = TrcUtil.getCurrentTime();

            if (debugLoopTimeEnabled)
            {
                if (prevTimeSliceStartTime != null)
                {
                    globalTracer.traceInfo(
                        funcName, "Timeslice Interval = %.6fs\n", timeSliceStartTime - prevTimeSliceStartTime);
                }
            }
            prevTimeSliceStartTime = timeSliceStartTime;
            mainThreadWatchdog.sendHeartBeat();

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
                TrcUtil.recordModeStartTime();

                if (prevMode != RunMode.INVALID_MODE)
                {
                    //
                    // Execute all stop tasks for previous mode.
                    //
                    startTime = TrcUtil.getCurrentTime();
                    TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, prevMode);
                    stopTaskElapsedTime = TrcUtil.getCurrentTime() - startTime;
                    if (debugPerformanceEnabled)
                    {
                        globalTracer.traceInfo(funcName, "%s.stopTaskElapsedTime=%.6fs", prevMode, stopTaskElapsedTime);
                    }
                    //
                    // Stop previous mode.
                    //
                    startTime = TrcUtil.getCurrentTime();

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

                    stopModeElapsedTime = TrcUtil.getCurrentTime() - startTime;
                    if (debugPerformanceEnabled)
                    {
                        globalTracer.traceInfo(funcName, "%s.stopModeElapsedTime=%.6fs", prevMode, stopModeElapsedTime);
                    }
                    //
                    // Run robotStopMode for the previous mode.
                    //
                    startTime = TrcUtil.getCurrentTime();
                    robotStopMode(prevMode, currMode);
                    robotStopModeElapsedTime = TrcUtil.getCurrentTime() - startTime;
                    if (debugPerformanceEnabled)
                    {
                        globalTracer.traceInfo(funcName, "%s.robotStopModeElapsedTime=%.6fs", prevMode, robotStopModeElapsedTime);
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
                    startTime = TrcUtil.getCurrentTime();
                    robotStartMode(currMode, prevMode);
                    robotStartModeElapsedTime = TrcUtil.getCurrentTime() - startTime;
                    if (debugPerformanceEnabled)
                    {
                        globalTracer.traceInfo(funcName, "%s.robotStartModeElapsedTime=%.6fs", currMode, robotStartModeElapsedTime);
                    }
                    //
                    // Start current mode.
                    //
                    startTime = TrcUtil.getCurrentTime();

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

                    startModeElapsedTime = TrcUtil.getCurrentTime() - startTime;
                    if (debugPerformanceEnabled)
                    {
                        globalTracer.traceInfo(funcName, "%s.startModeElapsedTime=%.6fs", currMode, startModeElapsedTime);
                    }
                    //
                    // Execute all start tasks for current mode.
                    //
                    startTime = TrcUtil.getCurrentTime();
                    TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, currMode);
                    startTaskElapsedTime = TrcUtil.getCurrentTime() - startTime;
                    if (debugPerformanceEnabled)
                    {
                        globalTracer.traceInfo(funcName, "%s.startTaskElapsedTime=%.6fs", currMode, startTaskElapsedTime);
                    }
                }
                //
                // Reset all performance counters for the mode.
                //
                fastLoopCounter = 0;
                slowLoopCounter = 0;
                fastPrePeriodicTaskTotalElapsedTime = 0.0;
                fastPrePeriodicTaskMaxElapsedTime = 0.0;
                slowPrePeriodicTaskTotalElapsedTime = 0.0;
                slowPrePeriodicTaskMaxElapsedTime = 0.0;
                fastPeriodicTotalElapsedTime = 0.0;
                fastPeriodicMaxElapsedTime = 0.0;
                slowPeriodicTotalElapsedTime = 0.0;
                slowPeriodicMaxElapsedTime = 0.0;
                fastPostPeriodicTaskTotalElapsedTime = 0.0;
                fastPostPeriodicTaskMaxElapsedTime = 0.0;
                slowPostPeriodicTaskTotalElapsedTime = 0.0;
                slowPostPeriodicTaskMaxElapsedTime = 0.0;
                updatesTotalElapsedTime = 0.0;
                updatesMaxElapsedTime = 0.0;
                nextSlowLoopTime = timeSliceStartTime;
            }

            //
            // Run the time slice.
            //
            double modeElapsedTime = TrcUtil.getModeElapsedTime();
            double currTime = TrcUtil.getCurrentTime();
            boolean runSlowLoop = false;

            fastLoopCounter++;
            if (currTime >= nextSlowLoopTime)
            {
                runSlowLoop = true;
                nextSlowLoopTime = currTime + slowLoopInterval;
                slowLoopCounter++;
            }

            //
            // FastPrePeriodic
            //
            startTime = TrcUtil.getCurrentTime();
            TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.FAST_PREPERIODIC_TASK, currMode);
            elapsedTime = TrcUtil.getCurrentTime() - startTime;
            fastPrePeriodicTaskTotalElapsedTime += elapsedTime;
            if (elapsedTime > fastPrePeriodicTaskMaxElapsedTime) fastPrePeriodicTaskMaxElapsedTime = elapsedTime;
            if (debugPerformanceEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "%s.fastPrePeriodicTaskElapsedTime=%.6f/%.6f/%.6fs",
                    currMode, elapsedTime, fastPrePeriodicTaskTotalElapsedTime/fastLoopCounter,
                    fastPrePeriodicTaskMaxElapsedTime);
            }
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(funcName, "%s.fastPerPeriodicTasks took too long (%.3fs)",
                    currMode, elapsedTime);
            }
            //
            // SlowPrePeriodic
            //
            if (runSlowLoop)
            {
                startTime = TrcUtil.getCurrentTime();
                TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.SLOW_PREPERIODIC_TASK, currMode);
                elapsedTime = TrcUtil.getCurrentTime() - startTime;
                slowPrePeriodicTaskTotalElapsedTime += elapsedTime;
                if (elapsedTime > slowPrePeriodicTaskMaxElapsedTime) slowPrePeriodicTaskMaxElapsedTime = elapsedTime;
                if (debugPerformanceEnabled)
                {
                    globalTracer.traceInfo(
                        funcName, "%s.slowPrePeriodicTaskElapsedTime=%.6f/%.6f/%.6fs",
                        currMode, elapsedTime, slowPrePeriodicTaskTotalElapsedTime/slowLoopCounter,
                        slowPrePeriodicTaskMaxElapsedTime);
                }
                if (elapsedTime > taskTimeThreshold)
                {
                    globalTracer.traceWarn(funcName, "%s.slowPrePeriodicTasks took too long (%.3fs)",
                        currMode, elapsedTime);
                }
            }
            //
            // FastPeriodic
            //
            startTime = TrcUtil.getCurrentTime();
            if (currMode == RunMode.DISABLED_MODE && disabledMode != null)
            {
                disabledMode.fastPeriodic(modeElapsedTime);
            }
            else if (currMode == RunMode.TEST_MODE && testMode != null)
            {
                testMode.fastPeriodic(modeElapsedTime);
            }
            else if (currMode == RunMode.AUTO_MODE && autoMode != null)
            {
                autoMode.fastPeriodic(modeElapsedTime);
            }
            else if (currMode == RunMode.TELEOP_MODE && teleOpMode != null)
            {
                teleOpMode.fastPeriodic(modeElapsedTime);
            }
            elapsedTime = TrcUtil.getCurrentTime() - startTime;
            fastPeriodicTotalElapsedTime += elapsedTime;
            if (elapsedTime > fastPeriodicMaxElapsedTime) fastPeriodicMaxElapsedTime = elapsedTime;
            if (debugPerformanceEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "%s.fastPeriodicElapsedTime=%.6f/%.6f/%.6fs",
                    currMode, elapsedTime, fastPeriodicTotalElapsedTime/fastLoopCounter,
                    fastPeriodicMaxElapsedTime);
            }
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(funcName, "%s.fastPeriodic took too long (%.3fs)",
                    currMode, elapsedTime);
            }
            //
            // SlowPeriodic
            //
            if (runSlowLoop)
            {
                startTime = TrcUtil.getCurrentTime();
                if (currMode == RunMode.DISABLED_MODE)
                {
                    HAL.observeUserProgramDisabled();
                    if (disabledMode != null)
                    {
                        disabledMode.slowPeriodic(modeElapsedTime);
                    }
                }
                else if (currMode == RunMode.TEST_MODE)
                {
                    HAL.observeUserProgramTest();
                    if (testMode != null)
                    {
                        testMode.slowPeriodic(modeElapsedTime);
                    }
                }
                else if (currMode == RunMode.AUTO_MODE)
                {
                    HAL.observeUserProgramAutonomous();
                    if (autoMode != null)
                    {
                        autoMode.slowPeriodic(modeElapsedTime);
                    }
                }
                else if (currMode == RunMode.TELEOP_MODE)
                {
                    HAL.observeUserProgramTeleop();
                    if (teleOpMode != null)
                    {
                        teleOpMode.slowPeriodic(modeElapsedTime);
                    }
                }
                elapsedTime = TrcUtil.getCurrentTime() - startTime;
                slowPeriodicTotalElapsedTime += elapsedTime;
                if (elapsedTime > slowPeriodicMaxElapsedTime) slowPeriodicMaxElapsedTime = elapsedTime;
                if (debugPerformanceEnabled)
                {
                    globalTracer.traceInfo(
                        funcName, "%s.slowPeriodicElapsedTime=%.6f/%.6f/%.6fs",
                        currMode, elapsedTime, slowPeriodicTotalElapsedTime/slowLoopCounter,
                        slowPeriodicMaxElapsedTime);
                }
                if (elapsedTime > taskTimeThreshold)
                {
                    globalTracer.traceWarn(funcName, "%s.slowPeriodic took too long (%.3fs)",
                        currMode, elapsedTime);
                }
            }
            //
            // FastPostPeriodic
            //
            startTime = TrcUtil.getCurrentTime();
            TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK, currMode);
            elapsedTime = TrcUtil.getCurrentTime() - startTime;
            fastPostPeriodicTaskTotalElapsedTime += elapsedTime;
            if (elapsedTime > fastPostPeriodicTaskMaxElapsedTime) fastPostPeriodicTaskMaxElapsedTime = elapsedTime;
            if (debugPerformanceEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "%s.fastPostPeriodicTaskElapsedTime=%.6f/%.6f/%.6fs",
                    currMode, elapsedTime, fastPostPeriodicTaskTotalElapsedTime/fastLoopCounter,
                    fastPostPeriodicTaskMaxElapsedTime);
            }
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(funcName, "%s.fastPostPeriodicTasks took too long (%.3fs)",
                    currMode, elapsedTime);
            }
            //
            // SlowPostPeriodic
            //
            if (runSlowLoop)
            {
                startTime = TrcUtil.getCurrentTime();
                TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.SLOW_POSTPERIODIC_TASK, currMode);
                elapsedTime = TrcUtil.getCurrentTime() - startTime;
                slowPostPeriodicTaskTotalElapsedTime += elapsedTime;
                if (elapsedTime > slowPostPeriodicTaskMaxElapsedTime) slowPostPeriodicTaskMaxElapsedTime = elapsedTime;
                if (debugPerformanceEnabled)
                {
                    globalTracer.traceInfo(
                        funcName, "%s.slowPostPeriodicTaskElapsedTime=%.6f/%.6f/%.6fs",
                        currMode, elapsedTime, slowPostPeriodicTaskTotalElapsedTime/slowLoopCounter,
                        slowPostPeriodicTaskMaxElapsedTime);
                }
                if (elapsedTime > taskTimeThreshold)
                {
                    globalTracer.traceWarn(funcName, "%s.slowPostPeriodicTask took too long (%.3fs)",
                        currMode, elapsedTime);
                }
            }

            startTime = TrcUtil.getCurrentTime();

            SmartDashboard.updateValues();

            if (liveWindowEnabled)
            {
                LiveWindow.updateValues();
            }

            if (dashboardEnabled && runSlowLoop)
            {
                //
                // Only update dashboard running time at periodic rate.
                //
                dashboard.displayPrintf(0, "[%3d:%06.3f] %s",
                    (int)(modeElapsedTime/60), modeElapsedTime%60, currMode);
            }

            elapsedTime = TrcUtil.getCurrentTime() - startTime;
            updatesTotalElapsedTime += elapsedTime;
            if (elapsedTime > updatesMaxElapsedTime) updatesMaxElapsedTime = elapsedTime;
            if (debugPerformanceEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "%s.updatesElapsedTime=%.6f/%.6f/%.6fs",
                    currMode, elapsedTime, updatesTotalElapsedTime/slowLoopCounter, updatesTotalElapsedTime);
            }
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(funcName, "%s.updates took too long (%.3fs)",
                    currMode, elapsedTime);
            }

            //
            // Do house keeping statistics and keep fast loop timeslice timing.
            //
            if (fastLoopInterval > 0.0)
            {
                double timeSliceUsed = TrcUtil.getCurrentTime() - timeSliceStartTime;
                if (timeSliceUsed >= fastLoopInterval)
                {
                    globalTracer.traceWarn(funcName, "%s took too long (%.3fs)", currMode, timeSliceUsed);
                }
                else
                {
                    TrcUtil.sleep((long) ((fastLoopInterval - timeSliceUsed)*1000));
                }
            }
        }
    }   //startCompetition

    /**
     * Called to end the competition loop.
     */
    @Override
    public void endCompetition()
    {
        mainThreadWatchdog.unregister();
        mainThreadWatchdog = null;
    }   //endCompetition

    /**
     * This method prints the performance metrics of all loops and taska.
     *
     * @param tracer specifies the tracer to be used for printing the performance metrics.
     */
    public void printPerformanceMetrics(TrcDbgTrace tracer)
    {
        tracer.traceInfo(
            moduleName, "(%s->%s) Performance Metrics: FastLoopCount=%d, SlowLoopCount=%d",
            prevMode, currMode, fastLoopCounter, slowLoopCounter);
        tracer.traceInfo(
            moduleName,
            "RobotInit=%.6f, PrevModeStopTask=%.6f, PrevStopMode=%.6f, PrevRobotStopMode=%.6f, " +
            "RobotStartMode=%.6f, startMode=%.6f, startTask=%.6f",
            robotInitElapsedTime, stopTaskElapsedTime, stopModeElapsedTime, robotStopModeElapsedTime,
            robotStartModeElapsedTime, startModeElapsedTime, startTaskElapsedTime);
        tracer.traceInfo(
            moduleName, "FastPrePeriodicTask(Avg/Max)=%.6f/%.6f",
            fastPrePeriodicTaskTotalElapsedTime/fastLoopCounter, fastPrePeriodicTaskMaxElapsedTime);
        tracer.traceInfo(
            moduleName, "SlowPrePeriodicTask(Avg/Max)=%.6f/%.6f",
            slowPrePeriodicTaskTotalElapsedTime/slowLoopCounter, slowPrePeriodicTaskMaxElapsedTime);
        tracer.traceInfo(
            moduleName, "FastPeriodic(Avg/Max)=%.6f/%.6f",
            fastPeriodicTotalElapsedTime/fastLoopCounter, fastPeriodicMaxElapsedTime);
        tracer.traceInfo(
            moduleName, "SlowPeriodic(Avg/Max)=%.6f/%.6f",
            slowPeriodicTotalElapsedTime/slowLoopCounter, slowPeriodicMaxElapsedTime);
        tracer.traceInfo(
            moduleName, "fastPostPeriodicTask(Avg/Max)=%.6f/%.6f",
            fastPostPeriodicTaskTotalElapsedTime/fastLoopCounter, fastPostPeriodicTaskMaxElapsedTime);
        tracer.traceInfo(
            moduleName, "SlowPostPeriodicTask(Avg/Max)=%.6f/%.6f",
            slowPostPeriodicTaskTotalElapsedTime/slowLoopCounter, slowPostPeriodicTaskMaxElapsedTime);
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
