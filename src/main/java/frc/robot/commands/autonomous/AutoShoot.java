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
        public AutoShoot(Drivetrain drivetrain, Arm arm, PoseEstimation poseEstimation) {
                Pose2d shootPose = AllianceUtils.allianceToField(Constants.AutoConstants.CUBE_SHOOT_POSITION);
                this.addCommands(
                        // AllianceUtils.allianceToField(new Pose2d(new Translation2d(3.76, 4.86), new Rotation2d(180)))
                        new FollowTrajectoryToState(drivetrain, poseEstimation, new PathPoint(shootPose.getTranslation(), shootPose.getRotation(), shootPose.getRotation(), 2), true),
                        new InstantCommand(() -> arm.setTarget(Arm.State.High)),
                        new WaitCommand(0.25),
                        new InstantCommand(() -> arm.setRollerState(Rollers.State.Outtake)),
                        new InstantCommand(() -> arm.setRollerState(Rollers.State.Off)),
                        new InstantCommand(() -> arm.setTarget(Arm.State.Stowed))
                );
        }
}
