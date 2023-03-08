package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.Set;

public class AndReturnToStart implements Command {
    private final Drivetrain drivetrain;
    private final PoseEstimation poseEstimation;

    private final Command inner;
    private boolean innerEnded;

    private Pose2d start;
    private NavigateToPoint returnToStart;

    public AndReturnToStart(PoseEstimation poseEstimation, Drivetrain drivetrain, Command inner) {
        this.poseEstimation = poseEstimation;
        this.drivetrain = drivetrain;

        this.inner = inner;
    }

    @Override
    public void initialize() {
        start = poseEstimation.getEstimatedPose();

        inner.initialize();
    }

    @Override
    public void execute() {
        if (!inner.isFinished()) {
            inner.execute();
        } else if (!innerEnded) {
            inner.end(false);
            innerEnded = true;

            returnToStart = new NavigateToPoint(drivetrain, poseEstimation, () -> start);
            returnToStart.initialize();
            returnToStart.execute();
        } else {
            returnToStart.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return innerEnded && returnToStart.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (!innerEnded) {
            inner.end(interrupted);
        } else {
            returnToStart.end(interrupted);
        }
    }

    @Override
    public Set<Subsystem> getRequirements() {
        Set<Subsystem> requirements = inner.getRequirements();
        requirements.add(drivetrain);
        return requirements;
    }
}
