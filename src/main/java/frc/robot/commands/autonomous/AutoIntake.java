package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
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

    private static final FollowTrajectoryToState.FieldPartition gamePiecePartition = new FollowTrajectoryToState.FieldPartition(
            Constants.FieldConstants.PRESET_PIECE_X,
            .5,
            new FollowTrajectoryToState.Waypoint(new Translation2d(0, 1.55), Rotation2d.fromRadians(Math.PI), 1),
            new FollowTrajectoryToState.Waypoint(new Translation2d(0, 2.77), Rotation2d.fromRadians(Math.PI), 1),
            new FollowTrajectoryToState.Waypoint(new Translation2d(0, 4), Rotation2d.fromRadians(Math.PI), 1),
            new FollowTrajectoryToState.Waypoint(new Translation2d(0, 5.25), Rotation2d.fromRadians(Math.PI), 1)

    );

    public AutoIntake(Drivetrain drivetrain, PoseEstimation poseEstimation, Arm arm, int index, GamePiece piece, boolean avoidFieldElements) {
        super(
                () -> {
                    FollowTrajectoryToState driveCommand = new FollowTrajectoryToState(
                            drivetrain,
                            poseEstimation,
                            getTargetPoint(index, piece, poseEstimation.getEstimatedPose()),
                            new PathConstraints(4, 2),
                            avoidFieldElements,
                            gamePiecePartition);
                    driveCommand.addTimedEvent(
                            driveCommand.trajectory.getTotalTimeSeconds() - Constants.Arm.INTAKING_BUFFER_TIME,
                            new InstantCommand(
                                    () -> {
                                        arm.setGamePiece(piece);
                                        arm.setTemporaryAngleOffset(Rotation2d.fromRadians(0.25));
                                        arm.setRollerState(Rollers.State.Intake);
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

        Translation2d lastPose = FollowTrajectoryToState.partitions[0].queryWaypoint(
                initialPose,
                new Pose2d(
                        piecePose,
                        new Rotation2d()
                )
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
        ).withPrevControlLength(Constants.AutoConstants.INTAKE_CONTROL_HANDLE);
    }
}
