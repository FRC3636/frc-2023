package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Node {
    private final GamePiece nodeType;
    private final Level level;
    private final Column column;

    public Node(GamePiece nodeType, Level level, Column column) {
        this.nodeType = nodeType;
        this.level = level;
        this.column = column;
    }

    public Node(int node) {
        this(
                GamePiece.fromNodeId(node),
                Level.values()[node / 3],
                Column.values()[node % 3]
        );

    }

    public Level getLevel() {
        return level;
    }

    public GamePiece getNodeType() {
        return nodeType;
    }

    public Column getColumn() {
        return column;
    }

    public Transform2d getRobotOffset() {
        double x = 0;
        switch (level) {
            case High:
                x = nodeType == GamePiece.Cone ? Constants.Arm.HIGH_CONE_SCORING_DIST
                        : Constants.Arm.HIGH_CUBE_SCORING_DIST;
                break;
            case Mid:
                x = nodeType == GamePiece.Cone ? Constants.Arm.MID_CONE_SCORING_DIST
                        : Constants.Arm.MID_CUBE_SCORING_DIST;
                break;
            case Low:
                x = nodeType == GamePiece.Cone ? Constants.Arm.LOW_CONE_SCORING_DIST
                        : Constants.Arm.LOW_CUBE_SCORING_DIST;
        }
        return new Transform2d(new Translation2d(-x, 0.0),
                new Rotation2d(0));
    }

    public Pose2d getNodePose() {
        Translation2d[] nodes;

        Pose2d robotPose = RobotContainer.poseEstimation.getEstimatedPose();

        int grid = (robotPose.getY() > Constants.FieldConstants.Grids.GRID_BOUNDARIES[1]
                ? robotPose.getY() > Constants.FieldConstants.Grids.GRID_BOUNDARIES[2] ? 2 : 1
                : 0);

        switch (level) {
            case High:
                nodes = Constants.FieldConstants.Grids.highTranslations;
                break;
            case Mid:
                nodes = Constants.FieldConstants.Grids.midTranslations;
                break;
            case Low:
                nodes = Constants.FieldConstants.Grids.lowTranslations;
                break;
            default:
                nodes = new Translation2d[0];
                break;
        }

        return AllianceUtils.allianceToField(new Pose2d(nodes[grid * 3 + column.getIndex()], new Rotation2d(Math.PI)));
    }

    public Pose2d getRobotScoringPose() {
        return getNodePose().transformBy(getRobotOffset());
    }

    public static Node getClosestNode(Pose2d robotPose, Level armLevel, GamePiece currentPiece){
        if(currentPiece == GamePiece.Cube){
            return new Node(currentPiece, armLevel, Column.Cube);
        }else{
            Node leftNode = new Node(currentPiece, armLevel, Column.LeftCone);
            Node rightNode = new Node(currentPiece, armLevel, Column.RightCone);
            double leftDistance = robotPose.getTranslation().getDistance(leftNode.getRobotScoringPose().getTranslation());
            double rightDistance = robotPose.getTranslation().getDistance(leftNode.getRobotScoringPose().getTranslation());
            if (leftDistance < rightDistance){
                return leftNode;
            }else{
                return rightNode;
            }
        }
    }

    public enum Level {
        High,
        Mid,
        Low,
    }

    public enum Column {
        LeftCone(2),
        Cube(1),
        RightCone(0);

        private final int blueIndex;

        public int getIndex() {
            return DriverStation.getAlliance() == DriverStation.Alliance.Blue ? blueIndex : 2 - blueIndex;
        }

        Column(int blueIndex) {
            this.blueIndex = blueIndex;
        }
    }

}
