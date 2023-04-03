package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.GamePiece;

public class Rollers {
    private Rollers.State rollerState = Rollers.State.Off;
    private GamePiece gamePiece = GamePiece.Cone;

    private Ultrasonic ultrasonic = new Ultrasonic(Constants.Rollers.PING_CHANNEL, Constants.Rollers.ECHO_CHANNEL);

    private final CANSparkMax motor = new CANSparkMax(Constants.Rollers.ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private boolean holdingGamePiece = false;

    public Rollers() {
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.restoreFactoryDefaults();
        ultrasonic.setEnabled(true);
        Ultrasonic.setAutomaticMode(true);
        RobotContainer.armTab.addBoolean("Holding Game Piece", this::isHoldingGamePiece);

        RobotContainer.armTab.add("Ultrasonic", ultrasonic);
    }

    public void setRollerState(State rollerState) {
        this.rollerState = rollerState;
    }

    public double getGamePieceOffset() {
        if(!ultrasonic.isRangeValid() || gamePiece == GamePiece.Cube) {
            return 0;
        }
        return (Constants.Rollers.INTAKE_WIDTH / 2) - (Units.inchesToMeters(ultrasonic.getRangeInches()) + Constants.Rollers.CONE_WIDTH / 2);
    }

    public void setGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    public double getRollerSpeed() {
        return gamePiece == GamePiece.Cone ? rollerState.coneSpeed : rollerState.cubeSpeed;
    }

    public boolean isHoldingGamePiece() {
        return holdingGamePiece;
    }

    public void periodic() {
        motor.set(getRollerSpeed());
        if (getRollerSpeed() != 0) {
            holdingGamePiece = Math.abs(motor.getEncoder().getVelocity()) < Constants.Rollers.HOLDING_PIECE_VELOCITY;
        }

        SmartDashboard.putNumber("Ultrasonic Sensor", getGamePieceOffset());
    }

    public enum State {
        Intake(Constants.Rollers.INTAKE_CONE, Constants.Rollers.INTAKE_CUBE),
        Outtake(Constants.Rollers.OUTTAKE_CONE, Constants.Rollers.OUTTAKE_CUBE),
        Off(-0.1, 0);

        public final double coneSpeed;
        public final double cubeSpeed;

        State(double coneSpeed, double cubeSpeed) {
            this.coneSpeed = coneSpeed;
            this.cubeSpeed = cubeSpeed;
        }

    }
}
