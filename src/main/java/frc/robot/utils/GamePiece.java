package frc.robot.utils;

public enum GamePiece {
    Cone,
    Cube;

    public static GamePiece fromNodeId(final int nodeId) {
        return nodeId % 3 == 1 ? Cube : Cone;
    }

    public static GamePiece fromLevelAndColumn(final int level, final int column) {
        final int nodeId = level * 3 + column;
        return fromNodeId(nodeId);
    }
}
