package frc.robot.commands.alignment;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmMoveCommand;
import frc.robot.commands.pathgeneration.FollowTrajectoryToState;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.GenerateCommand;
import frc.robot.utils.Node;

import java.util.Set;
import java.util.function.Supplier;


//Moves the robot to the node returned by the specified node supplier.
public class AlignToSelectedNode implements Command {

    private Command command;

    private final Drivetrain drivetrain;
    private final Arm arm;
    private final PoseEstimation poseEstimation;

    private final Supplier<Node> targetNode;

    private final double pidDeadline;

    public AlignToSelectedNode(Drivetrain drivetrain, Arm arm, PoseEstimation poseEstimation, Supplier<Node> targetNode, double pidDeadline){
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.poseEstimation = poseEstimation;
        this.targetNode = targetNode;
        this.pidDeadline = pidDeadline;
    }

    public AlignToSelectedNode(Drivetrain drivetrain, Arm arm, PoseEstimation poseEstimation, Supplier<Node> targetNode) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.poseEstimation = poseEstimation;
        this.targetNode = targetNode;
        this.pidDeadline = Double.POSITIVE_INFINITY;
    }

    @Override
    public void initialize() {
        Arm.State targetArmState = Arm.State.getTargetFromNode(targetNode.get());
        DriveToNode driveCommand = new DriveToNode(this.drivetrain, this.poseEstimation, targetNode.get().getWithGamePiece(arm.getRollers().getGamePieceOffset()), pidDeadline);

        double armMoveDelay =
                driveCommand.trajectoryCommand.trajectory.getTotalTimeSeconds() -
                (ArmMoveCommand.generateProfile(targetArmState, arm).totalTime() + Constants.Arm.RAISING_BUFFER_TIME);

        if(AllianceUtils.getDistanceFromAlliance(poseEstimation.getEstimatedPose()) > Constants.FieldConstants.fieldLength / 2) {
            command = new InstantCommand();
            command.initialize();
            return;
        }
        if(
                AllianceUtils.getDistanceFromAlliance(poseEstimation.getEstimatedPose()) < Constants.Arm.SAFE_RAISING_DISTANCE
                && ArmMoveCommand.pathIntersectsChargeStation(targetArmState, arm)
        ) {
            Pose2d initial = poseEstimation.getEstimatedPose();
            Pose2d waypoint = new Pose2d(
                    AllianceUtils.isBlue()?
                            Constants.Arm.SAFE_RAISING_DISTANCE :
                            Constants.FieldConstants.fieldLength - Constants.Arm.SAFE_RAISING_DISTANCE,
                    ((initial.getY() + targetNode.get().getRobotScoringPose().getY()) / 2),
                    AllianceUtils.allianceToField(Rotation2d.fromRadians(Math.PI))
            );

            FollowTrajectoryToState waypointCommand = new FollowTrajectoryToState(drivetrain, poseEstimation,
                    new PathPoint(
                            waypoint.getTranslation(),
                            Rotation2d.fromDegrees(targetNode.get().getRobotScoringPose().getY() > initial.getY() ? 90 : 270),
                            waypoint.getRotation(),
                            0.5
                    ),
                    true
            );

            command = waypointCommand.andThen(
                            new GenerateCommand(
                                    () -> new DriveToNode(this.drivetrain, this.poseEstimation, targetNode.get().getWithGamePiece(arm.getRollers().getGamePieceOffset()), pidDeadline),
                                    Set.of(drivetrain)
                            )
            ).alongWith(
                    new InstantCommand(() -> {
                        arm.setTarget(targetArmState);
                    })
            );
        }
        else if(
                armMoveDelay < 0
        ) {
            command = driveCommand.beforeStarting(
                    new WaitCommand(-armMoveDelay)
                            .alongWith(new InstantCommand(() -> arm.setTarget(targetArmState)))
            );
        }
        else {
            command = driveCommand;

            driveCommand.getTrajectoryCommand().addTimedEvent(
                    armMoveDelay,
                    new InstantCommand(() -> arm.setTarget(targetArmState))
            );

            driveCommand.getTrajectoryCommand().addConditionalEvent(
                    () -> AllianceUtils.getDistanceFromAlliance(poseEstimation.getEstimatedPose()) < Constants.Arm.AUTO_RAISING_DISTANCE,
                    new InstantCommand(() -> arm.setTarget(targetArmState))
            );
        }

        command.initialize();
    }

    @Override
    public void execute(){
        command.execute();
    }

    @Override
    public void end(boolean terminated){
        command.end(terminated);
    }

    @Override
    public boolean isFinished(){
        return command.isFinished();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
