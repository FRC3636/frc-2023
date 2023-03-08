package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Node;

import java.util.function.Supplier;

public class AlignToNode extends NavigateToPoint {

    public AlignToNode(Drivetrain drivetrain, PoseEstimation poseEstimation, Node targetNode) {
        super(drivetrain, poseEstimation, () -> targetNode.getNodePose().transformBy(targetNode.getRobotOffset()));
    }
}
