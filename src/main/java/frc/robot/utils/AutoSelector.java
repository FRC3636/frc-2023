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
import org.json.simple.JSONArray;
import org.json.simple.JSONValue;
import org.json.simple.parser.JSONParser;

import java.util.Collections;
import java.util.Map;
import java.util.function.Function;

public class AutoSelector implements Sendable {
    static Map<String, Function<String[], Command>> eventMap = Map.of(
            "print", (arguments) -> new InstantCommand(() -> System.out.println("print event triggered: " + arguments[0])),
            "score", (arguments) -> new AutoScore(
                    RobotContainer.drivetrain,
                    RobotContainer.arm,
                    RobotContainer.poseEstimation,
                    () -> new Node(GamePiece.valueOf(arguments[0]), Node.Level.valueOf(arguments[1]), Node.Column.valueOf(arguments[2]))
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

//        new JSONArray(auto)
//
//        for (String autoEvent : autoEvents) {
//            String event = autoEvent.split(":")[0].replaceAll("\"", "").trim();
//            String arguments = autoEvent.split(":")[1].trim().
//            autoCommand.addCommands();
//        }

        return autoCommand;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
