package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.alignment.AlignToSelectedNode;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Rollers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Node;

import java.util.function.Supplier;

//Move to a node, position the arm and outtake the current game piece
public class AutoScore extends SequentialCommandGroup {
    public AutoScore(Drivetrain drivetrain, Arm arm, PoseEstimation poseEstimation, Supplier<Node> targetNode) {
        super(
                new InstantCommand(() -> arm.setTargetFromNode(targetNode.get())),
                new AlignToSelectedNode(drivetrain, poseEstimation, targetNode),
                new InstantCommand(() -> arm.setRollerState(Rollers.State.Outtake)),
                new WaitCommand(0.5),
                new InstantCommand(() -> arm.setRollerState(Rollers.State.Off))
        );
    }
}