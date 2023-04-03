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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;

import java.util.*;
import java.util.function.Supplier;
import java.util.stream.Stream;

//Uses PathPlanner to move the robot to the specified Pose2d
public class FollowTrajectoryToState implements Command {
    protected final Drivetrain drivetrain;
    protected final PoseEstimation poseEstimation;
    protected final Map<Supplier<Boolean>, Command> events = new HashMap<>();
    protected final Timer timer = new Timer();

    protected final PathPoint target;
    public PathPlannerTrajectory trajectory;

    private PPSwerveControllerCommand swerveControllerCommand;

    public static final FieldPartition[] partitions = new FieldPartition[]{
            new FieldPartition(
                    3.9,
                    2,
                    new Waypoint(
                            new Translation2d(0, 0.75),
                            Rotation2d.fromDegrees(167.5)
                    ),
                    new Waypoint(
                            new Translation2d(0, 4.75),
                            Rotation2d.fromRotations(0.5)
                    )),
    };

    public FollowTrajectoryToState(Drivetrain drivetrain, PoseEstimation poseEstimation, PathPoint target, boolean avoidFieldElements) {
        this.drivetrain = drivetrain;
        this.poseEstimation = poseEstimation;
        this.target = target;

        if(avoidFieldElements) {
            trajectory = buildTrajectory(AutoConstants.DEFAULT_PATH_CONSTRAINTS, target, partitions);
        }
        else {
            trajectory = buildTrajectory(AutoConstants.DEFAULT_PATH_CONSTRAINTS, target);
        }
    }

    public FollowTrajectoryToState(Drivetrain drivetrain, PoseEstimation poseEstimation, PathPoint target, FieldPartition... fieldPartitions) {
        this.drivetrain = drivetrain;
        this.poseEstimation = poseEstimation;
        this.target = target;

        trajectory = buildTrajectory(AutoConstants.DEFAULT_PATH_CONSTRAINTS, target, fieldPartitions);
    }

    public FollowTrajectoryToState(Drivetrain drivetrain, PoseEstimation poseEstimation, PathPoint target, boolean avoidFieldElements, FieldPartition... fieldPartitions) {
        this(
                drivetrain,
                poseEstimation,
                target,
                avoidFieldElements ?
                        Stream.concat(Arrays.stream(partitions), Arrays.stream(fieldPartitions)).toArray(FieldPartition[]::new) :
                        fieldPartitions
        );
    }

    @Override
    public void initialize() {
        RobotContainer.field.getObject("Alignment Target").setPose(trajectory.getEndState().poseMeters);
        RobotContainer.field.getObject("Alignment Target").setTrajectory(trajectory);

        swerveControllerCommand = new PPSwerveControllerCommand(
                trajectory,
                poseEstimation::getEstimatedPose,
                new PIDController(
                        AutoConstants.P_TRANSLATION_CONTROLLER,
                        AutoConstants.I_TRANSLATION_CONTROLLER,
                        AutoConstants.D_TRANSLATION_CONTROLLER),
                new PIDController(
                        AutoConstants.P_TRANSLATION_CONTROLLER,
                        AutoConstants.I_TRANSLATION_CONTROLLER,
                        AutoConstants.D_TRANSLATION_CONTROLLER),
                new PIDController(AutoConstants.P_THETA_CONTROLLER, 0.0, 0.0),
                drivetrain::drive);

        timer.reset();
        timer.start();
        swerveControllerCommand.initialize();
    }

    protected PathPlannerTrajectory buildTrajectory(PathConstraints pathConstraints, PathPoint target, FieldPartition... fieldPartitions) {
        ArrayList<PathPoint> waypoints = new ArrayList<>();

        Pose2d initial = poseEstimation.getEstimatedPose();
        Translation2d initialV = drivetrain.getRelativeVelocity().rotateBy(initial.getRotation());

        waypoints.add(
                new PathPoint(
                        initial.getTranslation(),
                        initialV.getNorm() < 0.1 ?
                                target.position.minus(initial.getTranslation()).getAngle() :
                                initialV.getAngle(),
                        initial.getRotation(),
                        initialV.getNorm()
                )
        );

        RobotContainer.field.getObject("Alignment Start").setPose(new Pose2d(waypoints.get(0).position, waypoints.get(0).holonomicRotation));

        Arrays.stream(fieldPartitions).sorted(
                Collections.reverseOrder(Comparator.comparingDouble(partition -> Math.abs(partition.x - initial.getX())))
        ).forEach((fieldPartition) -> {
            Optional<PathPoint> waypoint = fieldPartition.queryWaypoint(
                    new Pose2d(waypoints.get(waypoints.size() - 1).position, waypoints.get(waypoints.size() - 1).holonomicRotation),
                    new Pose2d(target.position, target.holonomicRotation)
            );
            waypoint.ifPresent((point) -> {
                RobotContainer.field.getObject("Waypoint").setPose(new Pose2d(point.position, point.holonomicRotation));
                waypoints.add(point);
            });
        });

        waypoints.add(target);

        return PathPlanner.generatePath(
                pathConstraints,
                waypoints
        );
    }

    @Override
    public void execute() {
        swerveControllerCommand.execute();
        events.entrySet().removeIf((event) -> {
            if(event.getKey().get()) {
                event.getValue().schedule();
                return true;
            }
            return false;
        });
    }

    @Override
    public void end(boolean interrupted) {
        swerveControllerCommand.end(interrupted);
    }

    public void addTimedEvent(double time, Command command) {
        events.put(() -> timer.hasElapsed(time), command);
    }

    public void addConditionalEvent(Supplier<Boolean> supplier, Command command) {
        events.put(supplier, command);
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
        private final Waypoint[] waypoints;
        private final double partitionWidth;

        public FieldPartition(double x,  double partitionWidth, Waypoint... waypoints) {
            this.x = x;
            this.waypoints = waypoints;
            this.partitionWidth = partitionWidth;
        }

        public Optional<PathPoint> queryWaypoint(Pose2d start, Pose2d end) {
            // find intersection with partition, or return empty
            double fieldX = AllianceUtils.allianceToField(x);

            double t = (fieldX - start.getX()) / (end.getX() - start.getX());
            if (0 > t || t > 1) return Optional.empty();

            double intersectionY = end.getY() * t + start.getY() * (1 - t);

            if (waypoints.length == 0) return Optional.empty();

            Waypoint bestWaypoint = waypoints[0];

            for (Waypoint waypoint : waypoints) {
                double bestDistance = Math.abs(intersectionY - bestWaypoint.getPosition().getY());
                double distance = Math.abs(intersectionY - waypoint.getPosition().getY());

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

            return Optional.of(
                    new PathPoint(
                            new Translation2d(bestWaypoint.getPosition().getX() + fieldX, bestWaypoint.getPosition().getY()),
                            heading,
                            bestWaypoint.getHeadingOverride().orElse(start.getRotation().interpolate(end.getRotation(), 0.5)),
                            bestWaypoint.getVelocityOverride().orElse(-1.0)
                    ).withControlLengths(
                            getControlLength(Math.abs(start.getY() - bestWaypoint.getPosition().getY()), Math.abs(start.getX() - fieldX)),
                            getControlLength(Math.abs(end.getY() - bestWaypoint.getPosition().getY()), Math.abs(end.getX() - fieldX))
                    )
            );
        }

        private double getControlLength(double yOffset, double xOffset) {
            return Math.max(Math.min(partitionWidth * 1.25, Math.pow(yOffset * 2, 3) + xOffset / 3), 0.1);
        }
    }

    public static class Waypoint {
        private final Translation2d position;
        private final Optional<Rotation2d> headingOverride;
        private final Optional<Double> velocityOverride;

        public Waypoint(Translation2d position) {
            this.position = position;
            this.headingOverride = Optional.empty();
            velocityOverride = Optional.empty();
        }

        public Waypoint(Translation2d position, Rotation2d heading) {
            this.position = position;
            this.headingOverride = Optional.of(heading);
            velocityOverride = Optional.empty();
        }

        public Waypoint(Translation2d position, Rotation2d heading, double velocity) {
            this.position = position;
            this.headingOverride = Optional.of(heading);
            velocityOverride = Optional.of(velocity);
        }

        public Translation2d getPosition() {
            return AllianceUtils.mirrorByAlliance(position);
        }

        public Optional<Rotation2d> getHeadingOverride() {
            return headingOverride.map(AllianceUtils::mirrorByAlliance);
        }

        public Optional<Double> getVelocityOverride() {
            return velocityOverride;
        }
    }
}
