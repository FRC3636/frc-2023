package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Balance;
import frc.robot.commands.pathgeneration.FollowTrajectoryToPose;
import frc.robot.commands.pathgeneration.FollowTrajectoryToState;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.GenerateCommand;

import java.util.Set;

public class AutoBalance extends SequentialCommandGroup {
    public AutoBalance (Drivetrain drivetrain, PoseEstimation poseEstimation) {

        addCommands(
                new GenerateCommand(
                        () -> {
                            if (Math.abs(poseEstimation.getEstimatedPose().getX() - AllianceUtils.allianceToField(Constants.AutoConstants.BALANCE_X_POSITION)) < 0.8) {
                                return new FollowTrajectoryToPose(
                                        drivetrain,
                                        poseEstimation,
                                        new Pose2d(
                                                AllianceUtils.allianceToField(Constants.AutoConstants.LEAVE_COMMUNITY_DISTANCE),
                                                poseEstimation.getEstimatedPose().getY(),
                                                poseEstimation.getEstimatedPose().getRotation()
                                        ),
                                        true);
                            } else {
                                return new InstantCommand();
                            }
                        },
                        Set.of(drivetrain)
                )
        );
        addCommands(
                new GenerateCommand(
                        () -> {
                            try {
                                Pose2d targetXPosition = new Pose2d(AllianceUtils.allianceToField(Constants.AutoConstants.BALANCE_X_POSITION), poseEstimation.getEstimatedPose().getY(), poseEstimation.getEstimatedPose().getRotation());
                                PathPoint targetPoint = Constants.AutoConstants.BALANCING_OUTER_PARTITION.queryWaypoint(poseEstimation.getEstimatedPose(), targetXPosition).orElseGet(
                                        () -> Constants.AutoConstants.BALANCING_INNER_PARTITION.queryWaypoint(poseEstimation.getEstimatedPose(), targetXPosition).orElseThrow()
                                );
                                return new FollowTrajectoryToState(drivetrain, poseEstimation, targetPoint, false);
                            }
                            catch (Exception e) {
                                return new InstantCommand();
                            }
                        },
                        Set.of(drivetrain)
                )
        );

        addCommands(
                new Balance(drivetrain)
        );
    }
}
