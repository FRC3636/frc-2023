package frc.robot;

public enum ArmState {

    HighGoalCone(Constants.Shoulder.HIGH_CONE_ANGLE, GamePiece.Cone),
    HighGoalCube(Constants.Shoulder.HIGH_CUBE_ANGLE, GamePiece.Cube),
    MidGoalCone(Constants.Shoulder.MID_ANGLE, GamePiece.Cone),
    MidGoalCube(Constants.Shoulder.MID_ANGLE, GamePiece.Cube),
    LowGoalCone(Constants.Shoulder.INTAKE_CONE, GamePiece.Cone),
    LowGoalCube(Constants.Shoulder.STOWED_ANGLE, GamePiece.Cube),
    IntakeCone(Constants.Shoulder.INTAKE_CONE, GamePiece.Cone),
    IntakeCube(Constants.Shoulder.STOWED_ANGLE, GamePiece.Cube),
    Stowed(Constants.Shoulder.STOWED_ANGLE, GamePiece.Cube);


    public final double shoulderAngle;
    public final double wristAngle;
    public final GamePiece gamePiece;
    ArmState(double shoulderAngle, GamePiece gamePiece) {
        this.shoulderAngle = shoulderAngle;
        this.wristAngle = (gamePiece == GamePiece.Cone)? Constants.Wrist.CONE_ANGLE : Constants.Wrist.CUBE_ANGLE;
        this.gamePiece = gamePiece;
    }

    public static ArmState target = ArmState.Stowed;
    public enum GamePiece {
        Cone,
        Cube
    }
}
