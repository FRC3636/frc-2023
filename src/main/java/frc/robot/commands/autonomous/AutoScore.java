package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.alignment.AlignToSelectedNode;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Node;

import java.util.function.Supplier;

public class AutoScore extends SequentialCommandGroup {
    public AutoScore(Drivetrain drivetrain, PoseEstimation poseEstimation, Supplier<Node> targetNode){
        super(
                new InstantCommand(() -> Arm.State.setTargetFromNode(targetNode.get())),
                new AlignToSelectedNode(drivetrain, poseEstimation, targetNode)
        );
    }
}
