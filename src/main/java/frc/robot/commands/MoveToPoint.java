package frc.robot.commands;

import java.util.List;
import java.util.Set;

import javax.print.attribute.PrintServiceAttribute;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.PoseEstimation;

public class MoveToPoint extends PPSwerveControllerCommand {
    public MoveToPoint(Drivetrain drivetrain, PoseEstimation poseEstimation, Pose2d target) {
        super(
            makeTrajectory(poseEstimation, target),
            poseEstimation::getEstimatedPose,
            new PIDController(AutoConstants.PX_CONTROLLER, 0.0, 0.0),
            new PIDController(AutoConstants.PX_CONTROLLER, 0.0, 0.0),
            new PIDController(AutoConstants.P_THETA_CONTROLLER, 0.0, 0.0),
            (ChassisSpeeds speeds) -> drivetrain.drive(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                true
            )
        );
    }

    private static PathPlannerTrajectory makeTrajectory(PoseEstimation poseEstimation, Pose2d target) {
        Pose2d initial = poseEstimation.getEstimatedPose();
        Translation2d initialV = poseEstimation.getEstimatedVelocity();

        return PathPlanner.generatePath(
            new PathConstraints(
                AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
            ),
            new PathPoint(initial.getTranslation(), initialV.getAngle(), initial.getRotation(), initialV.getNorm()),
            new PathPoint(target.getTranslation(), target.getRotation(), target.getRotation())
        );
    }
}
