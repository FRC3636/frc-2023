package frc.robot.commands;

import java.util.Set;

import javax.sound.sampled.SourceDataLine;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;

public class AlignToNode implements Command {
    private final Drivetrain drivetrain;
    private final PoseEstimation poseEstimation;

    private PPSwerveControllerCommand swerveControllerCommand;

    public AlignToNode(Drivetrain drivetrain, PoseEstimation poseEstimation) {
        this.drivetrain = drivetrain;
        this.poseEstimation = poseEstimation;
    }

    @Override
    public void initialize() {
        Translation2d[] candidateNodes;
        switch (Arm.State.getTarget()) {
            case High:
                candidateNodes = FieldConstants.Grids.highTranslations;
                break;
            case Mid:
                candidateNodes = FieldConstants.Grids.midTranslations;
                break;
            case Low:
                candidateNodes = FieldConstants.Grids.lowTranslations;
                break;
            default:
                candidateNodes = new Translation2d[0];
                break;
        }

        PathPlannerTrajectory trajectory = null; 
        for (int i = 0; i < candidateNodes.length; i++) {
            Arm.State.GamePiece candidateNodePiece = i == 1 || i == 4 || i == 7 ? Arm.State.GamePiece.Cube : Arm.State.GamePiece.Cone;

            if (candidateNodePiece == Arm.State.getGamePiece()) {
                Translation2d candidateNode = candidateNodes[i];
                Pose2d candidateTarget = AllianceUtils.allianceToField(new Pose2d(candidateNode, new Rotation2d(Math.PI))
                                            .transformBy(AutoConstants.NODE_ALIGN_TRANSFORM));
                PathPlannerTrajectory candidateTrajectory = buildTrajectory(candidateTarget);

                if (trajectory == null || candidateTrajectory.getTotalTimeSeconds() < trajectory.getTotalTimeSeconds()) {
                    trajectory = candidateTrajectory;
                }
            }
        }

        if (trajectory == null) {
            trajectory = buildTrajectory(new Pose2d());
        }

        RobotContainer.field.getObject("Alignment Target").setPose(trajectory.getEndState().poseMeters);

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

    private PathPlannerTrajectory buildTrajectory(Pose2d target) {
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

        return trajectory;
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
