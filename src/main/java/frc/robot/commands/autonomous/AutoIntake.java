package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.pathgeneration.FollowTrajectoryToState;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Rollers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.GamePiece;
import frc.robot.utils.GenerateCommand;

import java.util.Set;

public class AutoIntake extends GenerateCommand {
    private final Arm arm;

    public AutoIntake(Drivetrain drivetrain, PoseEstimation poseEstimation, Arm arm, int index, GamePiece piece) {
        super(
                () -> {
                    FollowTrajectoryToState driveCommand = new FollowTrajectoryToState(
                            drivetrain,
                            poseEstimation,
                            getTargetPoint(index, piece, poseEstimation.getEstimatedPose()),
                            true);
                    driveCommand.addTimedEvent(
                            driveCommand.trajectory.getTotalTimeSeconds() - Constants.Arm.INTAKING_BUFFER_TIME,
                            new InstantCommand(
                                    () -> {
                                        arm.setGamePiece(piece);
                                        arm.setRollerState(Rollers.State.Intake);
                                        arm.setTemporaryAngleOffset(Rotation2d.fromRadians(0.5));
                                    }
                            )
                            );
                    return driveCommand;
                },
                Set.of(drivetrain)
        );

        this.arm = arm;
    }
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        arm.resetTemporaryAngleOffset();
        arm.setRollerState(Rollers.State.Off);
    }

    private static PathPoint getTargetPoint(int index, GamePiece piece, Pose2d initialPose) {


        Translation2d piecePose = AllianceUtils.allianceToField(
                new Translation2d(
                        Constants.FieldConstants.PRESET_PIECE_X,
                        Constants.FieldConstants.PRESET_PIECE_Y[index]
                )
        );

        Translation2d lastPose = FollowTrajectoryToState.chargingPadPartition.queryWaypoint(
                initialPose.getTranslation(),
                piecePose
        ).orElse(
                new PathPoint(
                        initialPose.getTranslation(),
                        new Rotation2d(),
                        new Rotation2d()
                )
        ).position;

        Pose2d targetPose =
                new Pose2d(
                        piecePose.interpolate(lastPose, Constants.AutoConstants.INTAKE_OFFSET.get(piece) / lastPose.getDistance(piecePose)),
                        piecePose.minus(lastPose).getAngle()
                );

        return new PathPoint(
                targetPose.getTranslation(),
                targetPose.getRotation(),
                targetPose.getRotation()
        ).withPrevControlLength(2);
    }
}
