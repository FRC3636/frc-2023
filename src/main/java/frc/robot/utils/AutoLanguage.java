package frc.robot.utils;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Balance;
import frc.robot.commands.autonomous.AutoBalance;
import frc.robot.commands.autonomous.AutoIntake;
import frc.robot.commands.autonomous.AutoScore;
import frc.robot.commands.autonomous.AutoShoot;
import frc.robot.commands.autonomous.DropGamePiece;
import frc.robot.commands.pathgeneration.FollowTrajectoryToPose;
import frc.robot.commands.pathgeneration.FollowTrajectoryToState;
import frc.robot.subsystems.arm.Rollers;

import java.util.Optional;
import java.util.Set;

public class AutoLanguage {
    public static Command compile(String source) {
        SequentialCommandGroup output = new SequentialCommandGroup();

        for (String statementSource : source.split(";")) {
            Command command = compileStatement(statementSource.strip());

            if (command != null) {
                output.addCommands(command);
            }
        }

        return output;
    }

    private static Command compileStatement(String source) {
        if (source.isEmpty()) return null;

        String[] tokens = source.split("\\s");

        switch (tokens[0]) {
            case "intake":
                GamePiece intakePiece = parseGamePiece(tokens[1]);
                int index = Integer.parseInt(tokens[2]) - 1;
                PathingMode mode = tokens.length > 3 ? parsePathingMode(tokens[3]) : PathingMode.AvoidObstacles;
                return new AutoIntake(RobotContainer.drivetrain, RobotContainer.poseEstimation, RobotContainer.arm, index, intakePiece, mode.avoidFieldElements);
            case "score":
                GamePiece scorePiece = parseGamePiece(tokens[1]);
                Optional<Integer> grid = parseGrid(tokens[2]);
                Node.Level level = parseNodeLevel(tokens[3]);
                Node.Column column = parseNodeColumn(tokens[4]);
                Node node = new Node(scorePiece, level, column, grid);
                return new InstantCommand(() -> RobotContainer.arm.setGamePiece(scorePiece)).
                        andThen(new AutoScore(RobotContainer.drivetrain, RobotContainer.arm, RobotContainer.poseEstimation, () -> node));
            case "shoot":
               return new AutoShoot(RobotContainer.drivetrain, RobotContainer.arm, RobotContainer.poseEstimation);
            case "balance":
                return new AutoBalance(RobotContainer.drivetrain, RobotContainer.poseEstimation);
            case "leave_community":
                PathingMode pathingMode = parsePathingMode(tokens[1]);
                Pose2d leaveCommunityPoint = new Pose2d(
                        AllianceUtils.allianceToField(Constants.AutoConstants.LEAVE_COMMUNITY_DISTANCE),
                        RobotContainer.poseEstimation.getEstimatedPose().getY(),
                        RobotContainer.poseEstimation.getEstimatedPose().getRotation()
                );

                return new GenerateCommand(
                        () -> new FollowTrajectoryToPose(RobotContainer.drivetrain, RobotContainer.poseEstimation, leaveCommunityPoint, pathingMode.avoidFieldElements),
                        Set.of(RobotContainer.drivetrain)
                );
            case "wait":
                double time = Double.parseDouble(tokens[1]);
                return new WaitCommand(time);
            case "drop":
                int dropIndex = Integer.parseInt(tokens[1]) - 1;
                return new DropGamePiece(dropIndex, RobotContainer.drivetrain);
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
        if (source.toLowerCase().equals("closest")) return Optional.empty();

        return Optional.of(Integer.parseInt(source) - 1);
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

    private static PathingMode parsePathingMode(String source) {
        switch (source) {
            case "avoid_obstacles":
                return PathingMode.AvoidObstacles;
            case "ignore_obstacles":
                return PathingMode.IgnoreObstacles;
            default:
                throw new RuntimeException("Attempted to parse invalid pathing mode: '" + source + "'");
        }
    }

    private enum PathingMode {
        AvoidObstacles(true),
        IgnoreObstacles(false);

        final boolean avoidFieldElements;

        PathingMode(boolean avoidFieldElements) {
            this.avoidFieldElements = avoidFieldElements;
        }
    }

    public static class AutoProgram implements Sendable {
        private String program;

        private Exception compilationError;
        private Command compilationOutput;

        public AutoProgram(String defaultProgram) {
            setProgram(defaultProgram);
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addStringProperty("Auto Program", this::getProgram, this::setProgram);
            builder.addStringProperty("Auto Compilation Error", () -> getCompilationError().map(Exception::getMessage).orElse(""), (e) -> {});
            builder.addBooleanProperty("Auto Compilation Good", () -> getCompilationOutput().isPresent(), (g) -> {});
        }

        public String getProgram() {
            return program;
        }

        public void setProgram(String program) {
            this.program = program;
            recompile();
        }

        public void recompile() {
            try {
                this.compilationOutput = compile(program);
                this.compilationError = null;
            } catch (Exception e) {
                this.compilationOutput = null;
                this.compilationError = e;
            }
        }

        public Optional<Command> getCompilationOutput() {
            return Optional.ofNullable(compilationOutput);
        }

        public Optional<Exception> getCompilationError() {
            return Optional.ofNullable(compilationError);
        }
    }
}

//hi silas, how is the auto code going. I am really hungry can i please have a snack. will the program be done before dcmp;. good job sila I am proud of you
