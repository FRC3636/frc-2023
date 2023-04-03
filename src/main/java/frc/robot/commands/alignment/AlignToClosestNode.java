package frc.robot.commands.alignment;

import frc.robot.RobotContainer;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.GenerateCommand;
import frc.robot.utils.Node;

import java.util.Set;

public class AlignToClosestNode extends GenerateCommand {
    public AlignToClosestNode(Drivetrain drivetrain, Arm arm, PoseEstimation poseEstimation) {
        super(() -> new DriveToNode(
                        drivetrain,
                        poseEstimation,
                        Node.getClosestNode(
                                RobotContainer.poseEstimation.getEstimatedPose(),
                                arm.getTarget().closestLevel(),
                                arm.getGamePiece()
                        ).getWithGamePiece(arm.getRollers().getGamePieceOffset()),
                        Double.POSITIVE_INFINITY
                ),
                Set.of(drivetrain)
        );
    }
}
