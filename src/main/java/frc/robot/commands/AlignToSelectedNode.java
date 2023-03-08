package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Node;

public class AlignToSelectedNode extends CommandBase {

    private AlignToNode innerCommand;

    private Drivetrain drivetrain;
    private PoseEstimation poseEstimation;

    public AlignToSelectedNode(Drivetrain drivetrain, PoseEstimation poseEstimation){
        this.innerCommand = new AlignToNode(drivetrain, poseEstimation, new Node(0));
        this.drivetrain = drivetrain;
        this.poseEstimation = poseEstimation;
    }

    public void setTargetNode(Node targetNode){
        this.innerCommand = new AlignToNode(this.drivetrain, this.poseEstimation, targetNode);
    }
}
