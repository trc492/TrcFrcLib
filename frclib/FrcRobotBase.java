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

    private static FrcRobotBase instance = null;

    private static long continuousLoopCounter = 0;
    private static long periodicLoopCounter = 0;
    private double robotInitElapsedTime = 0.0;
    private double stopTaskElapsedTime = 0.0;
    private double stopModeElapsedTime = 0.0;
    private double robotStopModeElapsedTime = 0.0;
    private double robotStartModeElapsedTime = 0.0;
    private double startModeElapsedTime = 0.0;
    private double startTaskElapsedTime = 0.0;
    private double preContinuousTaskTotalElapsedTime = 0.0;
    private double preContinuousTaskMaxElapsedTime = 0.0;
    private double prePeriodicTaskTotalElapsedTime = 0.0;
    private double prePeriodicTaskMaxElapsedTime = 0.0;
    private double runContinuousTotalElapsedTime = 0.0;
    private double runContinuousMaxElapsedTime = 0.0;
    private double runPeriodicTotalElapsedTime = 0.0;
    private double runPeriodicMaxElapsedTime = 0.0;
    private double postContinuousTaskTotalElapsedTime = 0.0;
    private double postContinuousTaskMaxElapsedTime = 0.0;
    private double postPeriodicTaskTotalElapsedTime = 0.0;
    private double postPeriodicTaskMaxElapsedTime = 0.0;
    private double updatesTotalElapsedTime = 0.0;
    private double updatesMaxElapsedTime = 0.0;

    private final String progName;
    private RobotMode teleOpMode = null;
    private RobotMode autoMode = null;
    private RobotMode testMode = null;
    private RobotMode disabledMode = null;
    private RunMode prevMode = RunMode.INVALID_MODE;
    private RunMode currMode = RunMode.INVALID_MODE;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param progName specifies the program name.
     */
    public FrcRobotBase(final String progName)
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

        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                globalTracer:
                new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.progName = progName;
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
     * This method returns the continuous loop counter. This is very useful for code to determine if it is called
     * multiple times in the same continuous loop. For example, it can be used to optimize sensor access so that if
     * the sensor is accessed in the same loop, there is no reason to create a new bus transaction to get "fresh"
     * data from the sensor.
     *
     * @return continuous loop counter value.
     */
    public static long getContinuousLoopCounter()
    {
        return continuousLoopCounter;
    }   //getContinuousLoopCounter

    /**
     * This method returns the periodic loop counter. This is very useful for code to determine if it is called
     * multiple times in the same periodic loop. For example, it can be used to optimize sensor access so that if
     * the sensor is accessed in the same loop, there is no reason to create a new bus transaction to get "fresh"
     * data from the sensor.
     *
     * @return periodic loop counter value.
     */
    public static long getPeriodicLoopCounter()
    {
        return periodicLoopCounter;
    }   //getPeriodicLoopCounter

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
     * Start the competition match. This specific startCompetition() implements "main loop" behavior like that of
     * the FRC control system in 2008 and earlier, with a primary (slow) loop that is called periodically, and a
     * "fast loop" (a.k.a. "spin loop") that is called as fast as possible with no delay between calls. This code
     * needs to track the order of the modes starting to ensure that everything happens in the right order. Repeatedly
     * run the correct method, either Autonomous or TeleOp when the robot is enabled. After running the correct method,
     * wait for some state to change, either the other mode starts or the robot is disabled. Then go back and wait for
     * the robot to be enabled again.
     */
    public void startCompetition()
    {
        final String funcName = "startCompetition";
        double startTime, elapsedTime;

        globalTracer.tracePrintf(
            FrcDbgLog.ESC_PREFIX + FrcDbgLog.SGR_FG_BLACK +
            FrcDbgLog.ESC_SEP + FrcDbgLog.SGR_BG_WHITE +
            FrcDbgLog.ESC_SUFFIX +
            "\n****************************************\n" +
            "Host Name: %s\n" +
            "  Program: %s\n"+
            "\n****************************************\n" +
            FrcDbgLog.ESC_NORMAL,
            getHostName(), progName);

        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Iterative);

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
        final double timesliceThreshold = 0.1;
        final double taskTimeThreshold = 0.05;

        while (true)
        {
            double timeSliceStart = TrcUtil.getCurrentTime();

            prevMode = currMode;
            //
            // Determine the current run mode.
            //
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

                continuousLoopCounter = 0;
                periodicLoopCounter = 0;
                preContinuousTaskTotalElapsedTime = 0.0;
                preContinuousTaskMaxElapsedTime = 0.0;
                prePeriodicTaskTotalElapsedTime = 0.0;
                prePeriodicTaskMaxElapsedTime = 0.0;
                runContinuousTotalElapsedTime = 0.0;
                runContinuousMaxElapsedTime = 0.0;
                runPeriodicTotalElapsedTime = 0.0;
                runPeriodicMaxElapsedTime = 0.0;
                postContinuousTaskTotalElapsedTime = 0.0;
                postContinuousTaskMaxElapsedTime = 0.0;
                postPeriodicTaskTotalElapsedTime = 0.0;
                postPeriodicTaskMaxElapsedTime = 0.0;
                updatesTotalElapsedTime = 0.0;
                updatesMaxElapsedTime = 0.0;
            }

            //
            // Run the time slice.
            //
            double modeElapsedTime = TrcUtil.getModeElapsedTime();
            boolean periodReady = isNewDataAvailable();
            continuousLoopCounter++;
            //
            // PreContinuous
            //
            startTime = TrcUtil.getCurrentTime();
            TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, currMode);
            elapsedTime = TrcUtil.getCurrentTime() - startTime;
            preContinuousTaskTotalElapsedTime += elapsedTime;
            if (elapsedTime > preContinuousTaskMaxElapsedTime) preContinuousTaskMaxElapsedTime = elapsedTime;
            if (debugPerformanceEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "%s.preContinuousTaskElapsedTime=%.6f/%.6f/%.6fs",
                    currMode, elapsedTime, preContinuousTaskTotalElapsedTime/continuousLoopCounter,
                    preContinuousTaskMaxElapsedTime);
            }
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(funcName, "%s.preContinuousTasks took too long (%.3fs)",
                    currMode, elapsedTime);
            }
            //
            // PrePeriodic
            //
            if (periodReady)
            {
                periodicLoopCounter++;
                startTime = TrcUtil.getCurrentTime();
                TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.PREPERIODIC_TASK, currMode);
                elapsedTime = TrcUtil.getCurrentTime() - startTime;
                prePeriodicTaskTotalElapsedTime += elapsedTime;
                if (elapsedTime > prePeriodicTaskMaxElapsedTime) prePeriodicTaskMaxElapsedTime = elapsedTime;
                if (debugPerformanceEnabled)
                {
                    globalTracer.traceInfo(
                        funcName, "%s.prePeriodicTaskElapsedTime=%.6f/%.6f/%.6fs",
                        currMode, elapsedTime, prePeriodicTaskTotalElapsedTime/periodicLoopCounter,
                        prePeriodicTaskMaxElapsedTime);
                }
                if (elapsedTime > taskTimeThreshold)
                {
                    globalTracer.traceWarn(funcName, "%s.prePeriodicTasks took too long (%.3fs)",
                        currMode, elapsedTime);
                }
            }
            //
            // Continuous
            //
            startTime = TrcUtil.getCurrentTime();
            if (currMode == RunMode.DISABLED_MODE && disabledMode != null)
            {
                disabledMode.runContinuous(modeElapsedTime);
            }
            else if (currMode == RunMode.TEST_MODE && testMode != null)
            {
                testMode.runContinuous(modeElapsedTime);
            }
            else if (currMode == RunMode.AUTO_MODE && autoMode != null)
            {
                autoMode.runContinuous(modeElapsedTime);
            }
            else if (currMode == RunMode.TELEOP_MODE && teleOpMode != null)
            {
                teleOpMode.runContinuous(modeElapsedTime);
            }
            elapsedTime = TrcUtil.getCurrentTime() - startTime;
            runContinuousTotalElapsedTime += elapsedTime;
            if (elapsedTime > runContinuousMaxElapsedTime) runContinuousMaxElapsedTime = elapsedTime;
            if (debugPerformanceEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "%s.runContinuousElapsedTime=%.6f/%.6f/%.6fs",
                    currMode, elapsedTime, runContinuousTotalElapsedTime/continuousLoopCounter,
                    runContinuousMaxElapsedTime);
            }
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(funcName, "%s.runContinuous took too long (%.3fs)",
                    currMode, elapsedTime);
            }
            //
            // Periodic
            //
            if (periodReady)
            {
                startTime = TrcUtil.getCurrentTime();
                if (currMode == RunMode.DISABLED_MODE)
                {
                    HAL.observeUserProgramDisabled();
                    if (disabledMode != null)
                    {
                        disabledMode.runPeriodic(modeElapsedTime);
                    }
                }
                else if (currMode == RunMode.TEST_MODE)
                {
                    HAL.observeUserProgramTest();
                    if (testMode != null)
                    {
                        testMode.runPeriodic(modeElapsedTime);
                    }
                }
                else if (currMode == RunMode.AUTO_MODE)
                {
                    HAL.observeUserProgramAutonomous();
                    if (autoMode != null)
                    {
                        autoMode.runPeriodic(modeElapsedTime);
                    }
                }
                else if (currMode == RunMode.TELEOP_MODE)
                {
                    HAL.observeUserProgramTeleop();
                    if (teleOpMode != null)
                    {
                        teleOpMode.runPeriodic(modeElapsedTime);
                    }
                }
                elapsedTime = TrcUtil.getCurrentTime() - startTime;
                runPeriodicTotalElapsedTime += elapsedTime;
                if (elapsedTime > runPeriodicMaxElapsedTime) runPeriodicMaxElapsedTime = elapsedTime;
                if (debugPerformanceEnabled)
                {
                    globalTracer.traceInfo(
                        funcName, "%s.runPeriodicElapsedTime=%.6f/%.6f/%.6fs",
                        currMode, elapsedTime, runPeriodicTotalElapsedTime/periodicLoopCounter,
                        runPeriodicMaxElapsedTime);
                }
                if (elapsedTime > taskTimeThreshold)
                {
                    globalTracer.traceWarn(funcName, "%s.runPeriodic took too long (%.3fs)",
                        currMode, elapsedTime);
                }
            }
            //
            // PostContinuous
            //
            startTime = TrcUtil.getCurrentTime();
            TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, currMode);
            elapsedTime = TrcUtil.getCurrentTime() - startTime;
            postContinuousTaskTotalElapsedTime += elapsedTime;
            if (elapsedTime > postContinuousTaskMaxElapsedTime) postContinuousTaskMaxElapsedTime = elapsedTime;
            if (debugPerformanceEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "%s.postContinuousTaskElapsedTime=%.6f/%.6f/%.6fs",
                    currMode, elapsedTime, postContinuousTaskTotalElapsedTime/continuousLoopCounter,
                    postContinuousTaskMaxElapsedTime);
            }
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(funcName, "%s.postContinuousTasks took too long (%.3fs)",
                    currMode, elapsedTime);
            }
            //
            // PostPeriodic
            //
            if (periodReady)
            {
                startTime = TrcUtil.getCurrentTime();
                TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.POSTPERIODIC_TASK, currMode);
                elapsedTime = TrcUtil.getCurrentTime() - startTime;
                postPeriodicTaskTotalElapsedTime += elapsedTime;
                if (elapsedTime > postPeriodicTaskMaxElapsedTime) postPeriodicTaskMaxElapsedTime = elapsedTime;
                if (debugPerformanceEnabled)
                {
                    globalTracer.traceInfo(
                        funcName, "%s.postPeriodicTaskElapsedTime=%.6f/%.6f/%.6fs",
                        currMode, elapsedTime, postPeriodicTaskTotalElapsedTime/periodicLoopCounter,
                        postPeriodicTaskMaxElapsedTime);
                }
                if (elapsedTime > taskTimeThreshold)
                {
                    globalTracer.traceWarn(funcName, "%s.postPeriodicTask took too long (%.3fs)",
                        currMode, elapsedTime);
                }
            }

            startTime = TrcUtil.getCurrentTime();

            SmartDashboard.updateValues();

            if (liveWindowEnabled)
            {
                LiveWindow.updateValues();
            }

            if (dashboardEnabled && periodReady)
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
                    currMode, elapsedTime, updatesTotalElapsedTime/periodicLoopCounter, updatesTotalElapsedTime);
            }
            if (elapsedTime > taskTimeThreshold)
            {
                globalTracer.traceWarn(funcName, "%s.updates took too long (%.3fs)",
                    currMode, elapsedTime);
            }

            //
            // Do house keeping statistics.
            //
            double timeSliceUsed = TrcUtil.getCurrentTime() - timeSliceStart;
            if (timeSliceUsed > timesliceThreshold)
            {
                globalTracer.traceWarn(funcName, "%s took too long (%.3fs)", currMode, timeSliceUsed);
            }
        }
    }   //startCompetition

    /**
     * Called to end the competition loop. I don't think we need to implement this.
     */
    @Override
    public void endCompetition()
    {
        // empty
    }

    /**
     * This method prints the performance metrics of all loops and taska.
     *
     * @param tracer specifies the tracer to be used for printing the performance metrics.
     */
    public void printPerformanceMetrics(TrcDbgTrace tracer)
    {
        tracer.traceInfo(
            moduleName, "%s Performance Metrics: ContinuousLoop=%d, PeriodicLoop=%d",
            continuousLoopCounter, periodicLoopCounter);
        tracer.traceInfo(
            moduleName,
            "RobotInit=%.6f, PrevModeStopTask=%.6f, PrevStopMode=%.6f, PrevRobotStopMode=%.6f, " +
            "RobotStartMode=%.6f, startMode=%.6f, startTask=%.6f",
            robotInitElapsedTime, stopTaskElapsedTime, stopModeElapsedTime, robotStopModeElapsedTime,
            robotStartModeElapsedTime, startModeElapsedTime, startTaskElapsedTime);
        tracer.traceInfo(
            moduleName, "PreContinuousTask(Avg/Max)=%.6f/%.6f",
            preContinuousTaskTotalElapsedTime/continuousLoopCounter, preContinuousTaskMaxElapsedTime);
        tracer.traceInfo(
            moduleName, "PrePeriodicTask(Avg/Max)=%.6f/%.6f",
            prePeriodicTaskTotalElapsedTime/periodicLoopCounter, prePeriodicTaskMaxElapsedTime);
        tracer.traceInfo(
            moduleName, "RunContinuous(Avg/Max)=%.6f/%.6f",
            runContinuousTotalElapsedTime/continuousLoopCounter, runContinuousMaxElapsedTime);
        tracer.traceInfo(
            moduleName, "RunPeriodic(Avg/Max)=%.6f/%.6f",
            runPeriodicTotalElapsedTime/periodicLoopCounter, runPeriodicMaxElapsedTime);
        tracer.traceInfo(
            moduleName, "PostContinuousTask(Avg/Max)=%.6f/%.6f",
            postContinuousTaskTotalElapsedTime/continuousLoopCounter, postContinuousTaskMaxElapsedTime);
        tracer.traceInfo(
            moduleName, "PostPeriodicTask(Avg/Max)=%.6f/%.6f",
            postPeriodicTaskTotalElapsedTime/periodicLoopCounter, postPeriodicTaskMaxElapsedTime);
        tracer.traceInfo(
            moduleName, "Updates(Avg/Max)=%.6f/%.6f",
            updatesTotalElapsedTime/periodicLoopCounter, updatesMaxElapsedTime);

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
