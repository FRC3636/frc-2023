package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.AutoScore;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.Map;
import java.util.function.Function;

public class AutoSelector implements Sendable {
    static Map<String, Function<String[], Command>> eventMap = Map.of(
            "print", (string) -> new InstantCommand(() -> System.out.println("print event triggered: " + string[0])),
            "score", (string) -> new AutoScore(
                    RobotContainer.drivetrain,
                    RobotContainer.arm,
                    RobotContainer.poseEstimation,
                    () -> new Node(GamePiece.valueOf(string[0]), Node.Level.valueOf(string[1]), Node.Column.valueOf(string[2]))
            )
//            "intake", new InstantCommand(() -> {
//                RobotContainer.arm.setRollerState(Rollers.State.Intake);
//            }),
//            "stow", new InstantCommand(() -> {
//                RobotContainer.arm.setRollerState(Rollers.State.Off);
//                RobotContainer.arm.setTarget(Arm.State.Stowed);
//            }),
//            "balance", new SequentialCommandGroup(
//                    new GenerateCommand( () ->
//                            new FollowTrajectoryToPose(
//                                    RobotContainer.drivetrain,
//                                    RobotContainer.poseEstimation,
//                                    AllianceUtils.allianceToField(new Pose2d(3.7, 2.9, new Rotation2d(Math.PI)))
//                            )
//                    ),
//                    new AutoBalance(RobotContainer.drivetrain))
    );

    public static Command makeAutoCommand(Drivetrain drivetrain, PoseEstimation poseEstimation, String auto) {
        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        String[] autoEvents = auto.toLowerCase().trim().split(",");

        for (String autoEvent : autoEvents) {
            String event = autoEvent.split(":")[0].replaceAll("\"", "");
            autoCommand.addCommands();
        }

        return autoCommand;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
