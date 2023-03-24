package frc.robot.utils;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.AutoBalance;
import frc.robot.commands.autonomous.AutoIntake;
import frc.robot.commands.autonomous.AutoScore;
import frc.robot.commands.pathgeneration.FollowTrajectoryToPose;
import frc.robot.commands.pathgeneration.FollowTrajectoryToState;

import java.nio.file.Path;
import java.util.Optional;
import java.util.Set;

public class AutoLanguage {
    public static Command compile(String source) {
        SequentialCommandGroup output = new SequentialCommandGroup();

        for (String statementSource : source.split(";")) {
            output.addCommands(compileStatement(statementSource.strip()));
        }

        return output;
    }

    private static Command compileStatement(String source) {
        String[] tokens = source.split("\\s");

        switch (tokens[0]) {
            case "intake":
                GamePiece intakePiece = parseGamePiece(tokens[1]);
                int index = Integer.parseInt(tokens[2]);
                return new AutoIntake(RobotContainer.drivetrain, RobotContainer.poseEstimation, RobotContainer.arm, index, intakePiece);
            case "score":
                GamePiece scorePiece = parseGamePiece(tokens[1]);
                Optional<Integer> grid = parseGrid(tokens[2]);
                Node.Level level = parseNodeLevel(tokens[3]);
                Node.Column column = parseNodeColumn(tokens[4]);
                Node node = new Node(scorePiece, level, column, grid);
                return new AutoScore(RobotContainer.drivetrain, RobotContainer.arm, RobotContainer.poseEstimation, () -> node);
            case "balance":
                boolean avoidObstacles = parsePathingMode(tokens[1]);

                Pose2d balanceStartPose = AllianceUtils.allianceToField(Constants.AutoConstants.BALANCE_STARTING_POINT_ALLIANCE_RELATIVE);
                PathPoint balanceStartPoint = new PathPoint(balanceStartPose.getTranslation(), balanceStartPose.getRotation());

                return new SequentialCommandGroup(
                        new GenerateCommand(
                                () -> new FollowTrajectoryToState(RobotContainer.drivetrain, RobotContainer.poseEstimation, balanceStartPoint, avoidObstacles),
                                Set.of(RobotContainer.drivetrain)
                        ),
                        new AutoBalance(RobotContainer.drivetrain)
                );
            default:
                throw new RuntimeException("Attempted to compile invalid statement of type: '" + tokens[0] + "'");
        }
    }

    private static GamePiece parseGamePiece(String source) {
        switch (source.toLowerCase()) {
            case "cone":
                return GamePiece.Cone;
            case "cube":
                return GamePiece.Cube;
            default:
                throw new RuntimeException("Attempted to parse invalid game piece: '" + source + "'");
        }
    }

    private static Optional<Integer> parseGrid(String source) {
        switch (source.toLowerCase()) {
            case "left":
                return Optional.of(0);
            case "middle":
            case "center":
                return Optional.of(1);
            case "right":
                return Optional.of(2);
            case "closest":
                return Optional.empty();
            default:
                throw new RuntimeException("Attempted to parse invalid grid: '" + source + "'");
        }
    }

    private static Node.Level parseNodeLevel(String source) {
        switch (source.toLowerCase()) {
            case "low":
                return Node.Level.Low;
            case "mid":
                return Node.Level.Mid;
            case "high":
                return Node.Level.High;
            default:
                throw new RuntimeException("Attempted to parse invalid node level: '" + source + "'");
        }
    }

    private static Node.Column parseNodeColumn(String source) {
        switch (source.toLowerCase()) {
            case "cone_left":
                return Node.Column.LeftCone;
            case "cube":
                return Node.Column.Cube;
            case "cone_right":
                return Node.Column.RightCone;
            default:
                throw new RuntimeException("Attempted to parse invalid node column: '" + source + "'");
        }
    }

    private static boolean parsePathingMode(String source) {
        switch (source) {
            case "avoid_obstacles":
                return true;
            case "ignore_obstacles":
                return false;
            default:
                throw new RuntimeException("Attempted to parse invalid pathing mode: '" + source + "'");
        }
    }
}
