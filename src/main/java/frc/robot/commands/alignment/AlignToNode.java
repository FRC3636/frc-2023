package frc.robot.commands.alignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.NavigateToPoint;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Node;

//Navigate to the point in front of the node specified
public class AlignToNode extends NavigateToPoint {

    public AlignToNode(Drivetrain drivetrain, PoseEstimation poseEstimation, Node targetNode) {
        super(drivetrain, poseEstimation, targetNode.getNodePose().transformBy(targetNode.getRobotOffset()));
    }
}