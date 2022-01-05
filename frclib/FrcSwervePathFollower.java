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

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcSwerveDriveBase;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;

import java.util.Arrays;

public class FrcSwervePathFollower
{
    private String instanceName;
    private Command swerveCommand;
    private SwerveDriveKinematics kinematics;
    private TrcPidController.PidCoefficients posPidCoeff;
    private TrcPidController.PidCoefficients headingPidCoeff;
    private final double maxRotVel;
    private final double maxRotAccel;
    private double maxWheelSpeed;
    private final TrcSwerveDriveBase driveBase;
    private double unitsPerMeter;
    private TrcEvent event;
    private double timeoutTime;
    private TrcTaskMgr.TaskObject driveTaskObj;

    public FrcSwervePathFollower(String instanceName, TrcSwerveDriveBase driveBase,
        TrcPidController.PidCoefficients posPidCoeff, TrcPidController.PidCoefficients headingPidCoeff,
        double maxRotVel, double maxRotAccel, double maxWheelSpeed)
    {
        this(instanceName, driveBase, posPidCoeff, headingPidCoeff, maxRotVel, maxRotAccel, maxWheelSpeed,
            1.0 / TrcUtil.METERS_PER_INCH);
    }

    public FrcSwervePathFollower(String instanceName, TrcSwerveDriveBase driveBase,
        TrcPidController.PidCoefficients posPidCoeff, TrcPidController.PidCoefficients headingPidCoeff,
        double maxRotVel, double maxRotAccel, double maxWheelSpeed, double unitsPerMeter)
    {
        this.instanceName = instanceName;
        this.driveBase = driveBase;
        this.posPidCoeff = posPidCoeff;
        this.headingPidCoeff = headingPidCoeff;
        this.maxRotVel = maxRotVel;
        this.maxRotAccel = maxRotAccel;
        this.maxWheelSpeed = maxWheelSpeed;
        this.unitsPerMeter = unitsPerMeter;
        double halfWidth = TrcUtil.METERS_PER_INCH * driveBase.getWheelBaseWidth() / 2;
        double halfLength = TrcUtil.METERS_PER_INCH * driveBase.getWheelBaseLength() / 2;
        // width is x for us, but -y for wpilib
        // length is y for us, but x for wpilib
        kinematics = new SwerveDriveKinematics(new Translation2d(halfLength, halfWidth),
            new Translation2d(halfLength, -halfWidth), new Translation2d(-halfLength, halfWidth),
            new Translation2d(-halfLength, -halfWidth));

        driveTaskObj = TrcTaskMgr.getInstance().createTask(instanceName + ".driveTaskObj", this::driveTask);
    }

    public void start(Trajectory... trajectories)
    {
        start(null, 0, trajectories);
    }

    /**
     * Start following the supplied trajectories back to back. If only one trajectory to follow, then just pass one.
     * The event will be signalled after all trajectories have been followed.
     * If the operation times out, it will be stopped and the event will be cancelled.
     * Use {@link FrcPath#createTrajectory} to create a trajectory.
     *
     * @param event        The event to signal when done. Set to null if unneeded.
     * @param timeout      Time in seconds to wait for the operation to finish. Set to 0 for no timeout.
     * @param trajectories The supplied trajectories to follow, in the absolute field reference frame.
     *                     The first point should be the robot's current pose.
     */
    public void start(TrcEvent event, double timeout, Trajectory... trajectories)
    {
        if (trajectories.length == 0)
        {
            throw new IllegalArgumentException("trajectories array cannot be empty!");
        }

        if (isActive())
        {
            cancel();
        }

        this.event = event;
        if (event != null)
        {
            event.clear();
        }
        timeoutTime = timeout == 0 ? Double.POSITIVE_INFINITY : TrcUtil.getCurrentTime() + timeout;
        PIDController xPid = new PIDController(posPidCoeff.kP, posPidCoeff.kI, posPidCoeff.kD);
        PIDController yPid = new PIDController(posPidCoeff.kP, posPidCoeff.kI, posPidCoeff.kD);
        // wpilib uses radians
        ProfiledPIDController headingPid = new ProfiledPIDController(headingPidCoeff.kP, headingPidCoeff.kI,
            headingPidCoeff.kD,
            new TrapezoidProfile.Constraints(Math.toRadians(maxRotVel), Math.toRadians(maxRotAccel)));
        swerveCommand = new SwerveControllerCommand(trajectories[0], this::getRobotPose, kinematics, xPid, yPid,
            headingPid, this::setSwerveStates);
        for (int i = 1; i < trajectories.length; i++)
        {
            swerveCommand = swerveCommand.andThen(
                new SwerveControllerCommand(trajectories[i], this::getRobotPose, kinematics, xPid, yPid, headingPid,
                    this::setSwerveStates));
        }
        swerveCommand = swerveCommand.andThen(this::stop);
        driveTaskObj.registerTask(TrcTaskMgr.TaskType.OUTPUT_TASK);
        swerveCommand.initialize();
    }

    public boolean isActive()
    {
        return driveTaskObj.isRegistered();
    }

    public void cancel()
    {
        swerveCommand.end(true);
        stop();
        if (event != null)
        {
            event.cancel();
        }
    }

    private void stop()
    {
        driveTaskObj.unregisterTask();
        driveBase.stop();
        if (event != null)
        {
            event.signal();
        }
    }

    @Override
    public String toString()
    {
        return instanceName;
    }

    private void driveTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        if (TrcUtil.getCurrentTime() < timeoutTime)
        {
            swerveCommand.execute();
        }
        else
        {
            swerveCommand.end(true);
        }
    }

    private Pose2d getRobotPose()
    {
        TrcPose2D pose = driveBase.getFieldPosition();
        // wpilib uses NWU coordinate frame, we use ENU. They use CCW radians, we use CW degrees
        return new Pose2d(pose.y / unitsPerMeter, -pose.x / unitsPerMeter, Rotation2d.fromDegrees(-pose.angle));
    }

    private void setSwerveStates(SwerveModuleState[] states)
    {
        // normalize wheel speeds
        SwerveDriveKinematics.normalizeWheelSpeeds(states, maxWheelSpeed / unitsPerMeter);
        // states has meters per second, and CCW angles
        double[][] velocities = Arrays.stream(states)
            .map(s -> new double[] { s.speedMetersPerSecond * unitsPerMeter / maxWheelSpeed, -s.angle.getDegrees() })
            .toArray(double[][]::new);
        driveBase.setModuleVelocities(velocities);
    }
}
