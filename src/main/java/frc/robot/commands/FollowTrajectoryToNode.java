package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.Node;

public class FollowTrajectoryToNode extends FollowTrajectoryToPoint{

    private final Node targetNode;

    public FollowTrajectoryToNode(Drivetrain drivetrain, PoseEstimation poseEstimation, Node targetNode) {
        super(drivetrain, poseEstimation, targetNode.getRobotScoringPose());

        this.targetNode = targetNode;
    }

    @Override
    protected PathPlannerTrajectory buildTrajectory(Pose2d target) {
        PathPlannerTrajectory initialTrajectory = super.buildTrajectory(target);

        double armMoveTime = ArmMoveCommand.generateProfile(Arm.State.getTargetFromNode(targetNode), RobotContainer.arm).totalTime();

        if(
                (initialTrajectory.getTotalTimeSeconds() > armMoveTime + 0.5 &&
                AllianceUtils.getDistanceFromAlliance(poseEstimation.getEstimatedPose()) > Constants.Arm.SAFE_RAISING_DISTANCE)
                || armMoveTime < 0.25
        ) {
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
