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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/**
 * This class modifies the behavior of the Trajectory class in a minor way.
 * The {@link FrcSwervePathFollower} class uses a {@link Trajectory} object to follow a path.
 * However, that class doesn't decouple heading from the velocity vector direction, which holonomic drivebases are capable of.
 * The path generation in {@link FrcPath#createHolonomicTrajectory} gets around this by modifying the heading values to
 * be the velocity vector directions instead of the target heading.
 * Since the path follower only uses the ending heading as the target (it doesn't use intermediate heading values) this wrapper class is the solution.
 * It acts identically to the Trajectory class most of the time, but when the final waypoint is being requested, the heading value is set as the target heading, not velocity direction.
 * This is basically bound to the implementation of {@link SwerveControllerCommand}, which isn't very elegant, but it works.
 */

public class FrcHolonomicTrajectory extends Trajectory
{
    private final double targetHeading;

    public FrcHolonomicTrajectory(Trajectory trajectory, double targetHeading)
    {
        super(trajectory.getStates());
        this.targetHeading = targetHeading;
    }

    @Override
    public State sample(double timeSeconds)
    {
        State state = super.sample(timeSeconds);
        // Return the pose with the target angle iff the time seconds is EXACTLY the end time
        if (timeSeconds == getTotalTimeSeconds())
        {
            // Create a new state object so the underlying list doesn't get modified. This is required so interpolation still works.
            Pose2d newPose = new Pose2d(state.poseMeters.getTranslation(), Rotation2d.fromDegrees(-targetHeading));
            state = new State(state.timeSeconds, state.velocityMetersPerSecond, state.accelerationMetersPerSecondSq,
                newPose, state.curvatureRadPerMeter);
        }
        return state;
    }
}
