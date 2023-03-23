package frc.robot.utils;

public interface PieceDependent<T> {
    T get(GamePiece piece);
}
