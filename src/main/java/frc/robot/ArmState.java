package frc.robot;

public enum ArmState {
    High(Constants.Shoulder.HIGH_CONE_ANGLE, Constants.Shoulder.HIGH_CUBE_ANGLE),
    Mid(Constants.Shoulder.MID_CONE_ANGLE, Constants.Shoulder.MID_CUBE_ANGLE),
    Low(Constants.Shoulder.INTAKE_CONE, Constants.Shoulder.INTAKE_CONE),
    Stowed(Constants.Shoulder.STOWED_ANGLE);


    private final double shoulderCubeAngle;
    private final double shoulderConeAngle;
    ArmState(double shoulderConeAngle, double shoulderCubeAngle) {
        this.shoulderCubeAngle = shoulderCubeAngle;
        this.shoulderConeAngle = shoulderConeAngle;
    }

    ArmState(double shoulderAngle) {
        this.shoulderCubeAngle = shoulderAngle;
        this.shoulderConeAngle = shoulderAngle;
    }

    public double getShoulderAngle() {
        return(gamePiece == GamePiece.Cone) ? shoulderConeAngle : shoulderCubeAngle;
    }

    public double getWristAngle() {
        return (gamePiece == GamePiece.Cone)? Constants.Wrist.CONE_ANGLE : Constants.Wrist.CUBE_ANGLE;
    }

    public static ArmState target = ArmState.Stowed;
    public static GamePiece gamePiece = GamePiece.Cube;
    public enum GamePiece {
        Cone,
        Cube
    }
}
