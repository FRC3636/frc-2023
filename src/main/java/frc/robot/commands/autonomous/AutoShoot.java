package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.pathgeneration.FollowTrajectoryToPose;
import frc.robot.commands.pathgeneration.FollowTrajectoryToState;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Rollers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.GenerateCommand;

import java.time.Instant;
import java.util.Set;

import com.pathplanner.lib.PathPoint;

public class AutoShoot extends GenerateCommand {
        public AutoShoot(Drivetrain drivetrain, boolean shouldShootInPlace) {
                super(
                        () -> { 
                                if (!shouldShootInPlace) {
                                        Pose2d shootPose = AllianceUtils.allianceToField(Constants.AutoConstants.CUBE_SHOOT_POSITION);
                                        FollowTrajectoryToState driveCommand = new FollowTrajectoryToState(
                                                drivetrain,
                                                RobotContainer.poseEstimation,
                                                new PathPoint(
                                                        shootPose.getTranslation(),
                                                        shootPose.getRotation(),
                                                        shootPose.getRotation()
                                                ),
                                                true);
                                        return driveCommand.andThen(
                                                new InstantCommand(() -> {
                                                        RobotContainer.arm.setTarget(Arm.State.High);
                                                }),
                                                new WaitCommand(1),
                                                new InstantCommand(() -> {
                                                        RobotContainer.arm.setRollerState(Rollers.State.Outtake);
                                                }),
                                                new InstantCommand(() -> RobotContainer.arm.setTarget(Arm.State.Stowed))
                                        );
                                } else {
                                        return new SequentialCommandGroup(
                                                new InstantCommand(() -> {
                                                        RobotContainer.arm.setTarget(Arm.State.High);
                                                }),
                                                new WaitCommand(1),
                                                new InstantCommand(() -> {
                                                        RobotContainer.arm.setRollerState(Rollers.State.Outtake);
                                                }),
                                                new InstantCommand(() -> RobotContainer.arm.setTarget(Arm.State.Stowed))  
                                        );
                                }
                                
                        }, 
                        Set.of(drivetrain)
                );
        }
}
