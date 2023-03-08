package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.function.Supplier;

public class NavigateToPoint extends SequentialCommandGroup {

    public NavigateToPoint(Drivetrain drivetrain, PoseEstimation poseEstimation, Pose2d target){
        super(
                new FollowTrajectoryToPoint(drivetrain, poseEstimation, target),
                new PIDDriveToPoint(drivetrain, poseEstimation, target)
        );
    }

}
