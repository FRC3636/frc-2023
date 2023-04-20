package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.pathgeneration.FollowTrajectoryToState;
import frc.robot.subsystems.arm.Rollers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.GenerateCommand;

import java.util.Set;
import java.util.function.Supplier;

public class DropGamePiece extends GenerateCommand {

    public DropGamePiece(int index, Drivetrain drivetrain) {
        super(
                () -> {
                    Pose2d dropPose = AllianceUtils.allianceToField(Constants.AutoConstants.GAME_PIECE_DROP_POSITIONS[index]);
                    FollowTrajectoryToState driveCommand = new FollowTrajectoryToState(
                            RobotContainer.drivetrain,
                            RobotContainer.poseEstimation,
                            new PathPoint(
                                    dropPose.getTranslation(),
                                    dropPose.getRotation(),
                                    dropPose.getRotation(),
                                    2
                            ).withPrevControlLength(1.5),
                            true);
                    return driveCommand.andThen(new InstantCommand(() -> RobotContainer.arm.setRollerState(Rollers.State.Outtake)));
                },
                Set.of(drivetrain)
        );
    }
}
