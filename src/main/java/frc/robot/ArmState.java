package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.ArmMoveCommand;

public enum ArmState {
    High(Constants.Shoulder.HIGH_CONE_ANGLE, Constants.Shoulder.HIGH_CUBE_ANGLE, Constants.Wrist.HIGH_CONE_ANGLE, Constants.Wrist.HIGH_CUBE_ANGLE),
    Mid(Constants.Shoulder.MID_CONE_ANGLE, Constants.Shoulder.MID_CUBE_ANGLE, Constants.Wrist.MID_CONE_ANGLE, Constants.Wrist.MID_CUBE_ANGLE),
    Low(Constants.Shoulder.INTAKE_CONE, Constants.Shoulder.INTAKE_CONE, Constants.Wrist.INTAKE_CONE, Constants.Wrist.INTAKE_CONE),
    Stowed(Constants.Shoulder.STOWED_ANGLE);

    private final double shoulderCubeAngle;
    private final double shoulderConeAngle;
    private final double wristConeAngle;
    private final double wristCubeAngle;
    ArmState(double shoulderConeAngle, double shoulderCubeAngle, double wristConeAngle, double wristCubeAngle) {
        this.shoulderCubeAngle = shoulderCubeAngle;
        this.shoulderConeAngle = shoulderConeAngle;
        this.wristConeAngle = wristConeAngle;
        this.wristCubeAngle = wristCubeAngle;
    }

    ArmState(double shoulderAngle) {
        this.shoulderCubeAngle = shoulderAngle;
        this.shoulderConeAngle = shoulderAngle;

        this.wristConeAngle = defautlWristAngle(GamePiece.Cone);
        this.wristCubeAngle = defautlWristAngle(GamePiece.Cube);
    }

    public double getShoulderAngle() {
        return(gamePiece == GamePiece.Cone) ? shoulderConeAngle : shoulderCubeAngle;
    }

    public double getWristAngle() {
        return(gamePiece == GamePiece.Cone) ? wristConeAngle : wristCubeAngle;
    }

    public double defautlWristAngle(GamePiece gamePiece) {
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
