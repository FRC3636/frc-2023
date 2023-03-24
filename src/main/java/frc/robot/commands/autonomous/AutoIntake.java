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

public class AutoIntake extends SequentialCommandGroup {
    public AutoIntake(Drivetrain drivetrain, PoseEstimation poseEstimation, Arm arm, int index, GamePiece piece) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            arm.setRollerState(Rollers.State.Intake);
                        }),
                        new SequentialCommandGroup(
                            new WaitUntilCommand(() -> AllianceUtils.getDistanceFromAlliance(poseEstimation.getEstimatedPose()) > Constants.Arm.SAFE_RAISING_DISTANCE),
                            new InstantCommand(() -> {
                                arm.setGamePiece(piece);
                                arm.setTarget(Arm.State.Stowed);
                            })
                        ),
                        new GenerateCommand(
                                () -> new FollowTrajectoryToState(drivetrain, poseEstimation, getTargetPoint(index, piece), true),
                                Set.of(drivetrain)
                        )
                ),
                new WaitCommand(0.5),
                new InstantCommand(() -> {
                    arm.setTarget(Arm.State.Stowed);
                    arm.setRollerState(Rollers.State.Off);
                })
        );
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
                targetPose.getRotation()
        );
    }
}
