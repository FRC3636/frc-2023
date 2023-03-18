package frc.robot.commands.alignment;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pathgeneration.FollowTrajectoryToNode;
import frc.robot.commands.PIDDriveToPoint;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Node;

//Navigate to the point in front of the node specified
public class DriveToNode extends SequentialCommandGroup {

    FollowTrajectoryToNode trajectoryCommand;

    public DriveToNode(Drivetrain drivetrain, PoseEstimation poseEstimation, Node targetNode) {
        trajectoryCommand = new FollowTrajectoryToNode(drivetrain, poseEstimation, targetNode);

        super.addCommands(
                trajectoryCommand,
                new PIDDriveToPoint(drivetrain, poseEstimation, targetNode.getRobotScoringPose())
        );
    }

    public double getEstimatedTotalTime() {
        return trajectoryCommand.trajectory.getTotalTimeSeconds();
    }
}