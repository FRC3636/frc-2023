package frc.robot.commands.pathgeneration;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class FollowTrajectoryToPose extends FollowTrajectoryToState {
    public FollowTrajectoryToPose(Drivetrain drivetrain, PoseEstimation poseEstimation, Pose2d target, boolean avoidFieldElements) {
        super(
                drivetrain,
                poseEstimation,
                new PathPoint(
                    target.getTranslation(),
                    target.getTranslation().minus(poseEstimation.getEstimatedPose().getTranslation()).getAngle(),
                    target.getRotation()
                ),
                avoidFieldElements
        );
    }

}
