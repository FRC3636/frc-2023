package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.function.Supplier;

//Moves the robot to a specified Pose2d
//Uses FollowTrajectoryToPoint for most of the movement,
//and PIDDriveToPoint at the end to correct for any error
public class NavigateToPoint extends SequentialCommandGroup {

    public NavigateToPoint(Drivetrain drivetrain, PoseEstimation poseEstimation, Pose2d target){
        super(
                new FollowTrajectoryToPose(drivetrain, poseEstimation, target),
                new PIDDriveToPoint(drivetrain, poseEstimation, target)
        );
    }

}
