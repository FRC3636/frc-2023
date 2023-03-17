package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;

import java.util.Set;

//Run the specified command, then return the robot to its position before the specified command was executed.
public class AndReturnToStart implements Command {
    private final Drivetrain drivetrain;
    private final PoseEstimation poseEstimation;
    private final Arm arm;

    private final Command inner;
    private boolean innerEnded;
    private final Timer timer = new Timer();

    private Pose2d startingPose;
    private Arm.State startingArmState;
    private FollowTrajectoryToState returnToStart;

    public AndReturnToStart(Drivetrain drivetrain, PoseEstimation poseEstimation, Arm arm, Command inner) {
        this.poseEstimation = poseEstimation;
        this.drivetrain = drivetrain;

        this.arm = arm;

        this.inner = inner;
    }

    @Override
    public void initialize() {
        startingPose = poseEstimation.getEstimatedPose();

        startingArmState = arm.getTarget();

        innerEnded = false;

        timer.reset();

        inner.initialize();
    }

    @Override
    public void execute() {

        if (!innerEnded) {
            if (!inner.isFinished()) {
                inner.execute();
            }
            else {
                inner.end(false);
                innerEnded = true;

                returnToStart = new FollowTrajectoryToPose(drivetrain, poseEstimation, startingPose);
                returnToStart.initialize();
                returnToStart.execute();

                timer.start();
            }
        } else {
            returnToStart.execute();
            if(
                    timer.get() > returnToStart.trajectory.getTotalTimeSeconds() - ArmMoveCommand.generateProfile(startingArmState, arm).totalTime() &&
                    arm.getTarget() != startingArmState &&
                    AllianceUtils.getDistanceFromAlliance(poseEstimation.getEstimatedPose()) > Constants.Arm.SAFE_RAISING_DISTANCE - 0.5
            ) {
                arm.setTarget(startingArmState);
            }
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
        return inner.getRequirements();
    }
}
