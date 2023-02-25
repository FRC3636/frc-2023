package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class MoveToPoint implements Command {
    private final Drivetrain drivetrain;
    private final PoseEstimation poseEstimation;

    private final Pose2d target;

    private PPSwerveControllerCommand swerveControllerCommand;

    public MoveToPoint(Drivetrain drivetrain, PoseEstimation poseEstimation, Pose2d target) {
        this.drivetrain = drivetrain;
        this.poseEstimation = poseEstimation;

        this.target = target;
    }

    @Override
    public void initialize() {
        Pose2d initial = poseEstimation.getEstimatedPose();
        Translation2d initialV = poseEstimation.getEstimatedVelocity();

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
            new PathConstraints(
                AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
            ),
            new PathPoint(initial.getTranslation(), initialV.getAngle(), initial.getRotation(), initialV.getNorm()),
            new PathPoint(target.getTranslation(), target.getRotation(), target.getRotation())
        );

        swerveControllerCommand = new PPSwerveControllerCommand(
            trajectory,
            poseEstimation::getEstimatedPose,
            new PIDController(AutoConstants.PX_CONTROLLER, 0.0, 0.0),
            new PIDController(AutoConstants.PX_CONTROLLER, 0.0, 0.0),
            new PIDController(AutoConstants.P_THETA_CONTROLLER, 0.0, 0.0),
            drivetrain::drive
        );
        
        swerveControllerCommand.initialize();
    }

    @Override
    public void execute() {
        swerveControllerCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        swerveControllerCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return swerveControllerCommand.isFinished();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
