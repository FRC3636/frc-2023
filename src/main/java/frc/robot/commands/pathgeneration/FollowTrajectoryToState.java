package frc.robot.commands.pathgeneration;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;

import java.util.Optional;
import java.util.Set;

//Uses PathPlanner to move the robot to the specified Pose2d
public class FollowTrajectoryToState implements Command {
    protected final Drivetrain drivetrain;
    protected final PoseEstimation poseEstimation;

    protected final PathPoint target;
    public PathPlannerTrajectory trajectory;

    private PPSwerveControllerCommand swerveControllerCommand;

    private static final FieldPartition chargingPadPartition = new FieldPartition(
            3.8,
            5,
            new PathPoint[] {
                    new PathPoint(
                        new Translation2d(0, 0.75),
                        new Rotation2d(),
                        Rotation2d.fromDegrees(150)
                ),
                new PathPoint(
                        new Translation2d(0, 4.75),
                        new Rotation2d(),
                        Rotation2d.fromDegrees(180)
                ),
            });

    public FollowTrajectoryToState(Drivetrain drivetrain, PoseEstimation poseEstimation, PathPoint target, boolean avoidFieldElements) {
        this.drivetrain = drivetrain;
        this.poseEstimation = poseEstimation;
        this.target = target;

        trajectory = buildTrajectory(target, avoidFieldElements);
    }

    @Override
    public void initialize() {
        RobotContainer.field.getObject("Alignment Target").setPose(trajectory.getEndState().poseMeters);
        RobotContainer.field.getObject("Alignment Target").setTrajectory(trajectory);

        swerveControllerCommand = new PPSwerveControllerCommand(trajectory, poseEstimation::getEstimatedPose, new PIDController(AutoConstants.P_TRANSLATION_PATH_CONTROLLER, 0.0, 0.0), new PIDController(AutoConstants.P_TRANSLATION_PATH_CONTROLLER, 0.0, 0.0), new PIDController(AutoConstants.P_THETA_PATH_CONTROLLER, 0.0, 0.0), drivetrain::drive);

        swerveControllerCommand.initialize();
    }

    protected PathPlannerTrajectory buildTrajectory(PathPoint target, boolean avoidFieldElements) {
        Pose2d initial = poseEstimation.getEstimatedPose();
        Translation2d initialV = poseEstimation.getEstimatedVelocity();

        PathPoint start = new PathPoint(
                initial.getTranslation(),
                initialV.getNorm() == 0 ?
                        target.position.minus(initial.getTranslation()).getAngle() :
                        initialV.getAngle(),
                initial.getRotation(),
                initialV.getNorm());

        RobotContainer.field.getObject("Alignment Start").setPose(new Pose2d(start.position, start.holonomicRotation));

        if(avoidFieldElements) {
            Optional<PathPoint> waypoint = chargingPadPartition.queryWaypoint(initial.getTranslation(), target.position);
            if (waypoint.isPresent()) {
                return PathPlanner.generatePath(
                        new PathConstraints(
                                AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                                AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
                        ),
                        start,
                        waypoint.get(),
                        target
                );
            }
        }
        return PathPlanner.generatePath(
                new PathConstraints(
                        AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                        AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
                ),
                start,
                target
        );
    }

    @Override
    public void execute() {
        swerveControllerCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        swerveControllerCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return swerveControllerCommand.isFinished();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }

    public static class FieldPartition {
        private final double x;
        private final PathPoint[] waypoints;
        private final double partitionWidth;

        public FieldPartition(double x,  double partitionWidth, PathPoint[] waypoints) {
            this.x = x;
            this.waypoints = waypoints;
            this.partitionWidth = partitionWidth;
        }

        public Optional<PathPoint> queryWaypoint(Translation2d start, Translation2d end) {
            // find intersection with partition, or return empty
            double fieldX = AllianceUtils.allianceToField(x);
            Pose2d[] fieldWaypoints = new Pose2d[waypoints.length];

            for (int i = 0; i < waypoints.length; i++) {
                fieldWaypoints[i] = AllianceUtils.mirrorPoseByAlliance(new Pose2d(waypoints[i].position, waypoints[i].holonomicRotation));
            }

            double t = (fieldX - start.getX()) / (end.getX() - start.getX());
            if (0 > t || t > 1) return Optional.empty();

            double intersectionY = end.getY() * t + start.getY() * (1 - t);

            if (waypoints.length == 0) return Optional.empty();

            Pose2d bestWaypoint = fieldWaypoints[0];

            for (Pose2d waypoint : fieldWaypoints) {
                double bestDistance = Math.abs(intersectionY - bestWaypoint.getY());
                double distance = Math.abs(intersectionY - waypoint.getY());

                if (distance < bestDistance) {
                    bestWaypoint = waypoint;
                }
            }

            Rotation2d heading;

            if (start.getX() < fieldX) {
                heading = Rotation2d.fromRotations(0);
            } else {
                heading = Rotation2d.fromRotations(0.5);
            }


            return Optional.of(new PathPoint(
                    new Translation2d(bestWaypoint.getX() + fieldX, bestWaypoint.getY()),
                    heading,
                    bestWaypoint.getRotation()
            ).withControlLengths(Math.min(partitionWidth / 2, Math.abs(start.getX() - fieldX)), partitionWidth / 2));
        }
    }
}
