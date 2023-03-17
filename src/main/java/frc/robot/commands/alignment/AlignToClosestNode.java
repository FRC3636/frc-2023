package frc.robot.commands.alignment;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.GenerateCommand;
import frc.robot.utils.Node;

public class AlignToClosestNode extends GenerateCommand {
    public AlignToClosestNode(Drivetrain drivetrain, Arm arm, PoseEstimation poseEstimation){
        super(() -> new DriveToNode(drivetrain, poseEstimation, Node.getClosestNode(RobotContainer.poseEstimation.getEstimatedPose(), arm.getTarget().closestLevel(), arm.getGamePiece())));
    }
}
