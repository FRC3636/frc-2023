package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.pathgeneration.FollowTrajectoryToState;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Rollers;
import frc.robot.subsystems.drivetrain.Drivetrain;


import com.pathplanner.lib.PathPoint;

public class AutoShoot extends SequentialCommandGroup {
        public AutoShoot(Drivetrain drivetrain, Arm arm, PoseEstimation poseEstimation) {
                this.addCommands(
                        new FollowTrajectoryToState(drivetrain, poseEstimation,new PathPoint(new Translation2d(3.76, 4.86), new Rotation2d(0), new Rotation2d(180), 4), null),
                        new InstantCommand(() -> arm.setTarget(Arm.State.Mid)),
                        new InstantCommand(() -> arm.moveHeightOffset(-0.1)),
                        new InstantCommand(() -> arm.setRollerState(Rollers.State.Outtake))
                );
        }
}
