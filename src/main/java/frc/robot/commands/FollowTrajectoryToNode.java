package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;

public class FollowTrajectoryToNode extends FollowTrajectoryToPoint{

    public FollowTrajectoryToNode(Drivetrain drivetrain, PoseEstimation poseEstimation, Pose2d target) {
        super(drivetrain, poseEstimation, target);
    }

    @Override
    protected PathPlannerTrajectory buildTrajectory(Pose2d target) {
        PathPlannerTrajectory initialTrajectory = super.buildTrajectory(target);

        if(
                initialTrajectory.getTotalTimeSeconds() > Constants.Arm.MAX_TIME + 0.5 &&
                AllianceUtils.getDistanceFromAlliance(poseEstimation.getEstimatedPose()) > Constants.Arm.SAFE_RAISING_DISTANCE) {
            return initialTrajectory;
        }

        Pose2d initial = poseEstimation.getEstimatedPose();
        Translation2d initialV = poseEstimation.getEstimatedVelocity();

        Pose2d waypoint = new Pose2d(
                AllianceUtils.isBlue()?
                        Constants.Arm.SAFE_RAISING_DISTANCE :
                        Constants.FieldConstants.fieldLength - Constants.Arm.SAFE_RAISING_DISTANCE,
                (initial.getY() + target.getY()) / 2,
                Rotation2d.fromDegrees(target.getY() > initial.getY() ? 90 : 270)
        );


        return PathPlanner.generatePath(
                new PathConstraints(
                        Constants.AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                        Constants.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
                ),
                new PathPoint(
                        initial.getTranslation(),
                        initialV.getNorm() == 0 ?
                                target.getTranslation().minus(initial.getTranslation()).getAngle() :
                                initialV.getAngle(),
                        initial.getRotation(),
                        initialV.getNorm()
                ),
                new PathPoint(
                        waypoint.getTranslation(),
                        waypoint.getRotation(),
                        initial.getRotation().interpolate(target.getRotation(), 0.5),
                        0.5
                ),
                new PathPoint(
                        target.getTranslation(),
                        target.getRotation(),
                        target.getRotation())
        );
    }
}
