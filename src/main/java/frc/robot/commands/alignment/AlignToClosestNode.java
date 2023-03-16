package frc.robot.commands.alignment;

import frc.robot.RobotContainer;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Node;

public class AlignToClosestNode extends AlignToSelectedNode{
    public AlignToClosestNode(Drivetrain drivetrain, PoseEstimation poseEstimation){
        super(drivetrain, poseEstimation, () -> Node.getClosestNode(RobotContainer.poseEstimation.getEstimatedPose(), Arm.State.closestLevel(), Arm.State.getGamePiece()));
    }
}
