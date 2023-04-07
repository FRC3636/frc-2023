package frc.robot.commands.alignment;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.PIDDriveToPoint;
import frc.robot.commands.pathgeneration.FollowTrajectoryToState;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.GenerateCommand;
import frc.robot.utils.Node;

import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class AlignToClosestElement extends GenerateCommand {
    public AlignToClosestElement(Drivetrain drivetrain, Arm arm, PoseEstimation poseEstimation) {
        super(() -> {
                    if (AllianceUtils.getDistanceFromAlliance(poseEstimation.getEstimatedPose()) < Constants.FieldConstants.fieldLength / 2) {
                        return new DriveToNode(
                                drivetrain,
                                poseEstimation,
                                Node.getClosestNode(
                                        RobotContainer.poseEstimation.getEstimatedPose(),
                                        arm.getTarget().closestLevel(),
                                        arm.getGamePiece()
                                ).getWithGamePiece(arm.getRollers().getGamePieceOffset()),
                                Double.POSITIVE_INFINITY
                        );
                    } else {
                        Transform2d tellerTransform = new Transform2d(new Translation2d(-Constants.Arm.TELLER_INTAKE_DIST, 0), new Rotation2d());
                        Transform2d slideTransform = new Transform2d(new Translation2d(-Constants.Arm.SLIDE_INTAKE_DIST, 0), new Rotation2d());



                        Pose2d targetPose;
                        double controlHandleLength;

                        if(RobotContainer.arm.getTarget() == Arm.State.Slide) {
                            targetPose = AllianceUtils.allianceToField(Constants.FieldConstants.SINGLE_SUBSTATION_POSE).transformBy(slideTransform);
                            controlHandleLength = Constants.AutoConstants.SLIDE_ALIGNMENT_CONTROL_HANDLE_LENGTH;
                        }
                        else if (RobotContainer.arm.getTarget() == Arm.State.Teller) {
                            targetPose = poseEstimation.getEstimatedPose()
                                    .nearest(
                                            Arrays.stream(Constants.FieldConstants.DOUBLE_SUBSTATION_POSES)
                                                    .map(AllianceUtils::allianceToField)
                                                    .map((tellerPose) -> tellerPose.transformBy(tellerTransform))
                                                    .collect(Collectors.toList())
                                    );
                            controlHandleLength = Constants.AutoConstants.TELLER_ALIGNMENT_CONTROL_HANDLE_LENGTH;
                        }
                        else {
                            return new InstantCommand();
                        }

                        return new SequentialCommandGroup(
                                new FollowTrajectoryToState(drivetrain, poseEstimation, new PathPoint(targetPose.getTranslation(), targetPose.getRotation(), targetPose.getRotation()).withPrevControlLength(
                                        Math.max(
                                                Math.min(
                                                        poseEstimation.getEstimatedPose().getTranslation().getDistance(targetPose.getTranslation()) * 2,
                                                        controlHandleLength),
                                                0.001
                                        )
                                ), false),
                                new PIDDriveToPoint(drivetrain, poseEstimation, targetPose)
                        );
                    }
                },
                Set.of(drivetrain)
        );
    }
}
