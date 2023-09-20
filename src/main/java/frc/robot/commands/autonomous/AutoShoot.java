package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.pathgeneration.FollowTrajectoryToPose;
import frc.robot.commands.pathgeneration.FollowTrajectoryToState;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Rollers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;

import com.pathplanner.lib.PathPoint;

public class AutoShoot extends SequentialCommandGroup {
        public AutoShoot(Drivetrain drivetrain, Arm arm, PoseEstimation poseEstimation, boolean shouldShootInPlace) {
                Pose2d shootPose = AllianceUtils.allianceToField(Constants.AutoConstants.CUBE_SHOOT_POSITION);
                // I'm probably doing this wrong. Someone please tell me how to do this right.
                if (!shouldShootInPlace) {
                        this.addCommands(
                                new FollowTrajectoryToPose(drivetrain, poseEstimation, shootPose, true)
                                
                                // new FollowTrajectoryToPose(drivetrain, poseEstimation, new PathPoint(shootPose.getTranslation(), new Rotation2d(), shootPose.getRotation()).withPrevControlLength(1.5), true)
                        );
                } 
                this.addCommands(
                        // AllianceUtils.allianceToField(new Pose2d(new Translation2d(3.76, 4.86), new Rotation2d(180)))
                        new InstantCommand(() -> arm.setTarget(Arm.State.High)),
                        new WaitCommand(1),
                        new InstantCommand(() -> arm.setRollerState(Rollers.State.Outtake)),
                        new WaitCommand(0.1),
                        new InstantCommand(() -> arm.setRollerState(Rollers.State.Off)),
                        new InstantCommand(() -> arm.setTarget(Arm.State.Stowed))
                );
        }
}
