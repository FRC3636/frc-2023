package frc.robot.commands.pathgeneration;

import com.pathplanner.lib.PathPoint;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Node;

public class FollowTrajectoryToNode extends FollowTrajectoryToState {

    public FollowTrajectoryToNode(Drivetrain drivetrain, PoseEstimation poseEstimation, Node targetNode) {
        super(drivetrain, poseEstimation, new PathPoint(
                targetNode.getRobotScoringPose().getTranslation(),
                targetNode.getRobotScoringPose().getRotation(),
                targetNode.getRobotScoringPose().getRotation()));
    }
}