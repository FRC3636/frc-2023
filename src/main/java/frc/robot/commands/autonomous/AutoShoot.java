package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.alignment.AlignToSelectedNode;
import frc.robot.commands.pathgeneration.FollowTrajectoryToState;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Rollers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.Node;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPoint;

public class AutoShoot extends SequentialCommandGroup {
        public AutoShoot(Drivetrain drivetrain, Arm arm, PoseEstimation poseEstimation) {
                this.addCommands(
                        new FollowTrajectoryToState(drivetrain, poseEstimation,new PathPoint(new Translation2d(3.76, 4.86), new Rotation2d(0), new Rotation2d(180), 4),null)
                new InstantCommand(
                        frc.robot.Constants.Rollers.SHOOT_CUBE;
                        )
                );
        }
}
/*
 * return new GenerateCommand(
 * () -> {
 * FollowTrajectoryToPose driveCommand = new FollowTrajectoryToPose(
 * RobotContainer.drivetrain,
 * RobotContainer.poseEstimation,
 * AllianceUtils.allianceToField(new Pose2d(4.47, 0.70,
 * Rotation2d.fromRotations(0.5))),
 * true);
 * driveCommand.addTimedEvent(1, new InstantCommand(() ->
 * RobotContainer.arm.setRollerState(Rollers.State.Outtake)));
 * return driveCommand;
 * },
 * Set.of(RobotContainer.drivetrain)
 * );
 */
/*
 * public class AutoScore extends SequentialCommandGroup {
 * public AutoScore(Drivetrain drivetrain, Arm arm, PoseEstimation
 * poseEstimation, Supplier<Node> targetNode){
 * this.addCommands(
 * new AlignToSelectedNode(drivetrain, arm, poseEstimation, targetNode, 0.75)
 * .raceWith(
 * new SequentialCommandGroup(
 * new WaitUntilCommand(() -> drivetrain.getRelativeVelocity().getNorm() > 0.2),
 * new WaitUntilCommand(() -> drivetrain.getRelativeVelocity().getNorm() <
 * 0.05),
 * new InstantCommand(() -> arm.setRollerState(Rollers.State.Outtake)),
 * new WaitCommand(0.25),
 * new InstantCommand(() -> arm.setRollerState(Rollers.State.Off))
 * )
 * ),
 * new InstantCommand(() ->
 * new WaitUntilCommand(
 * () ->
 * AllianceUtils.getDistanceFromAlliance(poseEstimation.getEstimatedPose()) >
 * Constants.Arm.SAFE_RAISING_DISTANCE
 * ).andThen(
 * new InstantCommand(() -> arm.setTarget(Arm.State.Stowed))
 * ).schedule()
 * )
 * );
 * }
 * }
 */