package frc.robot.commands.alignment;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.ArmMoveCommand;
import frc.robot.commands.pathgeneration.FollowTrajectoryToState;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
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
    private final Timer timer = new Timer();

    public AlignToSelectedNode(Drivetrain drivetrain, Arm arm, PoseEstimation poseEstimation, Supplier<Node> targetNode){
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.poseEstimation = poseEstimation;
        this.targetNode = targetNode;
    }

    @Override
    public void initialize(){
        Arm.State targetArmState = Arm.State.getTargetFromNode(targetNode.get());
        DriveToNode driveCommand = new DriveToNode(this.drivetrain, this.poseEstimation, targetNode.get());

        System.out.println(driveCommand.trajectoryCommand.trajectory.getTotalTimeSeconds());
        System.out.println(ArmMoveCommand.generateProfile(targetArmState, arm).totalTime() + Constants.Arm.RAISING_BUFFER_TIME);
        System.out.println(ArmMoveCommand.pathIntersectsChargeStation(targetArmState, arm));

        if(driveCommand.trajectoryCommand.trajectory.getTotalTimeSeconds() < ArmMoveCommand.generateProfile(targetArmState, arm).totalTime() + Constants.Arm.RAISING_BUFFER_TIME
                /*&& ArmMoveCommand.pathIntersectsChargeStation(targetArmState, arm)*/) {
            System.out.println("test");
            Pose2d initial = poseEstimation.getEstimatedPose();
            Pose2d waypoint = new Pose2d(
                    AllianceUtils.isBlue()?
                            Constants.Arm.SAFE_RAISING_DISTANCE :
                            Constants.FieldConstants.fieldLength - Constants.Arm.SAFE_RAISING_DISTANCE,
                    (initial.getY() + targetNode.get().getRobotScoringPose().getY()) / 2,
                    AllianceUtils.getAllianceToField(Rotation2d.fromRadians(Math.PI * 5 / 4))
            );

            FollowTrajectoryToState waypointCommand = new FollowTrajectoryToState(drivetrain, poseEstimation,
                    new PathPoint(
                            waypoint.getTranslation(),
                            Rotation2d.fromDegrees(targetNode.get().getRobotScoringPose().getY() > initial.getY() ? 90 : 270),
                            waypoint.getRotation(),
                            0.5
                    )
            );

            command = new ParallelCommandGroup(
                    driveCommand.beforeStarting(waypointCommand),
                    new WaitCommand(
                            driveCommand.trajectoryCommand.trajectory.getTotalTimeSeconds() -
                                    (ArmMoveCommand.generateProfile(targetArmState, arm).totalTime() + Constants.Arm.RAISING_BUFFER_TIME)
                    ).andThen(
                            new InstantCommand(() -> arm.setTarget(targetArmState))
                    )
            );

        }
        else {
            command = new ParallelCommandGroup(driveCommand,
                    (new SequentialCommandGroup(
                    new WaitCommand(
                            driveCommand.trajectoryCommand.trajectory.getTotalTimeSeconds() -
                                    (ArmMoveCommand.generateProfile(targetArmState, arm).totalTime() + Constants.Arm.RAISING_BUFFER_TIME)
                    ),
                    new InstantCommand(() -> arm.setTarget(targetArmState))
                    )
            ));
        }

        command.initialize();
    }

    @Override
    public void execute(){
        command.execute();
    }

    @Override
    public void end(boolean terminated){ command.end(terminated); }

    @Override
    public boolean isFinished(){
        return command.isFinished();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain, arm);
    }
}
