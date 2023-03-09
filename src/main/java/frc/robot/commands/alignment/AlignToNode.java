package frc.robot.commands.alignment;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FollowTrajectoryToNode;
import frc.robot.commands.PIDDriveToPoint;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Node;

//Navigate to the point in front of the node specified
public class AlignToNode extends SequentialCommandGroup {

    public AlignToNode(Drivetrain drivetrain, PoseEstimation poseEstimation, Node targetNode) {
        super(
                new FollowTrajectoryToNode(drivetrain, poseEstimation, targetNode.getNodePose().transformBy(targetNode.getRobotOffset())),
                new PIDDriveToPoint(drivetrain, poseEstimation, targetNode.getNodePose().transformBy(targetNode.getRobotOffset()))
        );
    }
}