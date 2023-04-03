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

    private static final FollowTrajectoryToState.FieldPartition gamePiecePartition = new FollowTrajectoryToState.FieldPartition(
            Constants.FieldConstants.PRESET_PIECE_X,
            .5,
            new FollowTrajectoryToState.Waypoint(new Translation2d(0, 1.55), Rotation2d.fromRadians(Math.PI), 1),
            new FollowTrajectoryToState.Waypoint(new Translation2d(0, 2.77), Rotation2d.fromRadians(Math.PI), 1),
            new FollowTrajectoryToState.Waypoint(new Translation2d(0, 4), Rotation2d.fromRadians(Math.PI), 1),
            new FollowTrajectoryToState.Waypoint(new Translation2d(0, 5.25), Rotation2d.fromRadians(Math.PI), 1)

    );

    public AutoIntake(Drivetrain drivetrain, PoseEstimation poseEstimation, Arm arm, int index, GamePiece piece) {
        super(
                () -> {
                    FollowTrajectoryToState driveCommand = new FollowTrajectoryToState(
                            drivetrain,
                            poseEstimation,
                            getTargetPoint(index, piece),
                            true,
                            gamePiecePartition);
                    driveCommand.addTimedEvent(
                            driveCommand.trajectory.getTotalTimeSeconds() - Constants.Arm.INTAKING_BUFFER_TIME,
                            new InstantCommand(
                                    () -> {
                                        arm.setGamePiece(piece);
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
        arm.setRollerState(Rollers.State.Off);
    }

    private static PathPoint getTargetPoint(int index, GamePiece piece) {


        Pose2d piecePose = AllianceUtils.allianceToField(new Pose2d(
                new Translation2d(
                        Constants.FieldConstants.PRESET_PIECE_X,
                        Constants.FieldConstants.PRESET_PIECE_Y[index]
                ),
                new Rotation2d()
        ));

        Pose2d targetPose = piecePose.transformBy(Constants.AutoConstants.INTAKE_OFFSET.get(piece));

        return new PathPoint(
                targetPose.getTranslation(),
                targetPose.getRotation(),
                targetPose.getRotation(),
                2
        ).withPrevControlLength(0.7);
    }
}
