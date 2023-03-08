package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Node;

import java.util.Set;

public class AlignToSelectedNode implements Command {

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

    @Override
    public void execute() {
        innerCommand.execute();
    }

    @Override
    public void initialize() {
        innerCommand.initialize();
    }

    @Override
    public boolean isFinished() {
        return innerCommand.isFinished();
    }

    @Override
    public void end(boolean terminated) {
        innerCommand.end(terminated);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
