package frc.robot.commands.alignment;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.alignment.AlignToNode;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Node;

import java.util.function.Supplier;

public class AlignToSelectedNode extends CommandBase {

    private AlignToNode innerCommand;

    private Drivetrain drivetrain;
    private PoseEstimation poseEstimation;

    private Supplier<Node> targetNode;

    public AlignToSelectedNode(Drivetrain drivetrain, PoseEstimation poseEstimation, Supplier<Node> targetNode){
        this.innerCommand = new AlignToNode(drivetrain, poseEstimation, new Node(0));
        this.drivetrain = drivetrain;
        this.poseEstimation = poseEstimation;
        this.targetNode = targetNode;
    }

    @Override
    public void execute(){
        innerCommand.execute();
    }

    @Override
    public void initialize(){
        this.innerCommand = new AlignToNode(this.drivetrain, this.poseEstimation, targetNode.get());
        innerCommand.initialize();
    }

    @Override
    public void end(boolean terminated){
        innerCommand.end(terminated);
    }

    @Override
    public boolean isFinished(){
        return innerCommand.isFinished();
    }
}
