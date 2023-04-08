package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.alignment.AlignToSelectedNode;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Rollers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.Node;

import java.util.function.Supplier;

//Move to a node, position the arm and outtake the current game piece
public class AutoScore extends SequentialCommandGroup {
    public AutoScore(Drivetrain drivetrain, Arm arm, PoseEstimation poseEstimation, Supplier<Node> targetNode){
        this.addCommands(
                new AlignToSelectedNode(drivetrain, arm, poseEstimation, targetNode, 0.75)
                        .raceWith(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> drivetrain.getRelativeVelocity().getNorm() > 0.05),
                                        new WaitUntilCommand(() -> drivetrain.getRelativeVelocity().getNorm() < 0.05),
                                        new InstantCommand(() -> arm.setRollerState(Rollers.State.Outtake)),
                                        new WaitCommand(0.25),
                                        new InstantCommand(() -> arm.setRollerState(Rollers.State.Off))
                                )
                        ),
                new InstantCommand(() ->
                        new WaitUntilCommand(
                                () -> AllianceUtils.getDistanceFromAlliance(poseEstimation.getEstimatedPose()) > Constants.Arm.SAFE_RAISING_DISTANCE
                        ).andThen(
                                new InstantCommand(() -> arm.setTarget(Arm.State.Stowed))
                        ).schedule()
                )
        );
    }
}