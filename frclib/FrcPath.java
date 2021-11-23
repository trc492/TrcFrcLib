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

package frclib;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import TrcCommonLib.trclib.TrcPath;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcSwerveDriveBase;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcWaypoint;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class FrcPath extends TrcPath
{
    public enum SplineType
    {
        HERMITE_QUINTIC, CLAMPED_CUBIC
    }

    public static Pose2d toWpiPose(TrcPose2D pose)
    {
        return new Pose2d(pose.y * TrcUtil.METERS_PER_INCH, -pose.x * TrcUtil.METERS_PER_INCH,
            Rotation2d.fromDegrees(-pose.angle));
    }

    /**
     * Create a {@link TrajectoryConfig} object to use for path creation. This config is specific to swerve drives.
     * It applies a max velocity and acceleration to the path to create a trapezoidal velocity profile.
     * It also uses the maximum steer velocity (rotation rate) of the swerve modules to limit the centripetal acceleration of the path.
     * This protects against the robot's velocity vector changing direction too fast for the modules to follow.
     * It also constrains the path such that none of the wheels will exceed the maximum wheel velocity.
     *
     * @param driveBase   The drivebase used for the path creation.
     * @param maxVel      The maximum velocity of the robot, in inches per second.
     * @param maxAccel    The maximum acceleration of the robot, in inches per second per second.
     * @param maxSteerVel The maximum steer velocity of the robot, in degrees per second.
     * @return A TrajectoryConfig to be used for trajectory configuration.
     */
    public static TrajectoryConfig createSwerveConfig(TrcSwerveDriveBase driveBase, double maxVel, double maxAccel,
        double maxSteerVel)
    {
        double maxVelMeters = maxVel * TrcUtil.METERS_PER_INCH;
        double maxAccelMeters = maxAccel * TrcUtil.METERS_PER_INCH;
        double halfWidth = TrcUtil.METERS_PER_INCH * driveBase.getWheelBaseWidth() / 2;
        double halfLength = TrcUtil.METERS_PER_INCH * driveBase.getWheelBaseLength() / 2;
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d(halfLength, halfWidth),
            new Translation2d(halfLength, -halfWidth), new Translation2d(-halfLength, halfWidth),
            new Translation2d(-halfLength, -halfWidth));
        TrajectoryConfig config = new TrajectoryConfig(maxVelMeters, maxAccelMeters);
        config.addConstraint(new CentripetalAccelerationConstraint(0.6 * maxVelMeters * Math.toRadians(maxSteerVel)));
        config.addConstraint(new SwerveDriveKinematicsConstraint(kinematics, maxVelMeters));
        return config;
    }

    /**
     * Create a {@link Trajectory} from a {@link TrcPath} object. Only the positions are preserved.
     * The velocities and accelerations will be defined by the {@link TrajectoryConfig}.
     * The trajectory headings will be modified, since heading refers to the velocity vector direction.
     * The returned Trajectory will be a {@link FrcHolonomicTrajectory} object, so the last sample will have the target heading as the heading value.
     * All other heading values refer to velocity direction.
     * <p>
     * This is meant for holonomic drivebases, where heading is decoupled from the velocity vector.
     *
     * @param path   The TrcPath object to use to create the Trajectory.
     * @param config This specifies the constraints and configurations to use when making the Trajectory.
     * @return A Trajectory object with the appropriate constraints.
     */
    public static Trajectory createHolonomicTrajectory(TrcPath path, TrajectoryConfig config)
    {
        if (path.getSize() < 2)
        {
            throw new IllegalArgumentException("Path must have at least two points!");
        }

        path = path.clone();

        TrcPose2D start = path.getWaypoint(0).getPositionPose();
        TrcPose2D next = path.getWaypoint(1).getPositionPose();
        double startTheta = Math.toDegrees(Math.atan2(next.x - start.x, next.y - start.y));

        TrcPose2D prev = path.getWaypoint(path.getSize() - 2).getPositionPose();
        TrcPose2D last = path.getLastWaypoint().getPositionPose();
        double endTheta = Math.toDegrees(Math.atan2(last.x - prev.x, last.y - prev.y));

        double targetHeading = last.angle;
        path.getWaypoint(0).heading = startTheta;
        path.getLastWaypoint().heading = endTheta;

        Trajectory trajectory = createTrajectory(path, config, SplineType.CLAMPED_CUBIC);
        return new FrcHolonomicTrajectory(trajectory, targetHeading);
    }

    /**
     * Create a {@link Trajectory} from a {@link TrcPath} object. Only the positions are preserved.
     * The velocities and accelerations will be defined by the {@link TrajectoryConfig}.
     * If the spline type is hermite quintic, then the intermediate headings will also be preserved.
     *
     * @param path   The TrcPath object to use to create the Trajectory.
     * @param config This specifies the constraints and configurations to use when making the Trajectory.
     * @param type   The type of spline to use. If cubic, the intermediate poses become "knot points", and only their positions will be preserved.
     *               If quintic, the intermediate poses' headings are preserved. More or less a moot point for holonomic drivebases.
     *               Keep in mind that some path followers won't honor intermediate headings.
     * @return A Trajectory object with the appropriate constraints. The trajectory should pass through every point, and start and end at the correct headings.
     */
    public static Trajectory createTrajectory(TrcPath path, TrajectoryConfig config, SplineType type)
    {
        List<Pose2d> poses = Arrays.stream(path.getAllWaypoints()).map(TrcWaypoint::getPositionPose)
            .map(FrcPath::toWpiPose).collect(Collectors.toList());
        Trajectory trajectory;
        switch (type)
        {
            case CLAMPED_CUBIC:
                List<Translation2d> positions = poses.subList(1, poses.size() - 1).stream().map(Pose2d::getTranslation)
                    .collect(Collectors.toList());
                trajectory = TrajectoryGenerator
                    .generateTrajectory(poses.get(0), positions, poses.get(poses.size() - 1), config);
                break;

            case HERMITE_QUINTIC:
                trajectory = TrajectoryGenerator.generateTrajectory(poses, config);
                break;

            default:
                throw new IllegalStateException("Unexpected SplineType value: " + type);
        }
        return trajectory;
    }

    public FrcPath(boolean inDegrees, TrcWaypoint... waypoints)
    {
        super(inDegrees, waypoints);
    }

    public FrcPath(TrcWaypoint... waypoints)
    {
        super(waypoints);
    }

    /**
     * Create a {@link Trajectory} from a {@link TrcPath} object. Only the positions are preserved.
     * The velocities and accelerations will be defined by the {@link TrajectoryConfig}.
     * If the spline type is hermite quintic, then the intermediate headings will also be preserved.
     *
     * @param config This specifies the constraints and configurations to use when making the Trajectory.
     * @param type   The type of spline to use. If cubic, the intermediate poses become "knot points", and only their positions will be preserved.
     *               If quintic, the intermediate poses' headings are preserved. More or less a moot point for holonomic drivebases.
     *               Keep in mind that some path followers won't honor intermediate headings.
     * @return A Trajectory object with the appropriate constraints. The trajectory should pass through every point, and start and end at the correct headings.
     */
    public Trajectory createTrajectory(TrajectoryConfig config, SplineType type)
    {
        return createTrajectory(this, config, type);
    }
}
