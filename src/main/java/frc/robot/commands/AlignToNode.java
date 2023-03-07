package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Node;

public class AlignToNode extends SequentialCommandGroup {

    public AlignToNode(Drivetrain drivetrain, PoseEstimation poseEstimation) {
        super(
                new FollowTrajectoryToPoint(drivetrain, poseEstimation, () -> Node.getTarget().getNodePose().transformBy(Node.getTarget().getRobotOffset())),
                new DriveToPoint(drivetrain, poseEstimation, () -> Node.getTarget().getNodePose().transformBy(Node.getTarget().getRobotOffset()))
        );
    }


}
