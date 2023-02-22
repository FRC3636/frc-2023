package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.ArmMoveCommand;

public enum ArmState {
    High(Constants.Shoulder.HIGH_CONE_ANGLE, Constants.Shoulder.HIGH_CUBE_ANGLE, Constants.Wrist.HIGH_CONE_ANGLE, Constants.Wrist.HIGH_CUBE_ANGLE),
    Mid(Constants.Shoulder.MID_CONE_ANGLE, Constants.Shoulder.MID_CUBE_ANGLE, Constants.Wrist.MID_CONE_ANGLE, Constants.Wrist.MID_CUBE_ANGLE),
    Low(Constants.Shoulder.INTAKE_CONE, Constants.Shoulder.INTAKE_CONE, Constants.Wrist.INTAKE_CONE, Constants.Wrist.MID_CUBE_ANGLE),
    Stowed(Constants.Shoulder.STOWED_ANGLE);

    private final Rotation2d shoulderCubeAngle;
    private final Rotation2d shoulderConeAngle;
    private final Rotation2d wristConeAngle;
    private final Rotation2d wristCubeAngle;
    ArmState(Rotation2d shoulderConeAngle, Rotation2d shoulderCubeAngle, Rotation2d wristConeAngle, Rotation2d wristCubeAngle) {
        this.shoulderCubeAngle = shoulderCubeAngle;
        this.shoulderConeAngle = shoulderConeAngle;
        this.wristConeAngle = wristConeAngle;
        this.wristCubeAngle = wristCubeAngle;
    }

    ArmState(Rotation2d shoulderAngle) {
        this.shoulderCubeAngle = shoulderAngle;
        this.shoulderConeAngle = shoulderAngle;

        this.wristConeAngle = defaultWristAngle(GamePiece.Cone);
        this.wristCubeAngle = defaultWristAngle(GamePiece.Cube);
    }

    public Rotation2d getShoulderAngle() {
        return(gamePiece == GamePiece.Cone) ? shoulderConeAngle : shoulderCubeAngle;
    }

    public Rotation2d getWristAngle() {
        return(gamePiece == GamePiece.Cone) ? wristConeAngle : wristCubeAngle;
    }

    public Rotation2d defaultWristAngle(GamePiece gamePiece) {
        return (gamePiece == GamePiece.Cone)? Constants.Wrist.CONE_ANGLE : Constants.Wrist.CUBE_ANGLE;
    }

    private static ArmState target = ArmState.Stowed;
    private static GamePiece gamePiece = GamePiece.Cube;

    public static ArmState getTarget() {
        return target;
    }

    public static void setTarget(ArmState target) {
        ArmState.target = target;
        new ArmMoveCommand(RobotContainer.shoulder, RobotContainer.wrist).schedule();
    }

    public static GamePiece getGamePiece() {
        return gamePiece;
    }

    public static void setGamePiece(GamePiece gamePiece) {
        ArmState.gamePiece = gamePiece;
        new ArmMoveCommand(RobotContainer.shoulder, RobotContainer.wrist).schedule();
    }

    public enum GamePiece {
        Cone,
        Cube
    }
}
