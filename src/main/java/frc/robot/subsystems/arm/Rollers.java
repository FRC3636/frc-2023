package frc.robot.subsystems.arm;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.GamePiece;

public class Rollers {
    private Rollers.State rollerState = Rollers.State.Off;
    private GamePiece gamePiece = GamePiece.Cube;

    private TimeOfFlight TOF = new TimeOfFlight(0);

    private final CANSparkMax motor = new CANSparkMax(Constants.Rollers.ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private boolean holdingGamePiece = false;

    public Rollers() {
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.restoreFactoryDefaults();
        TOF.setRangingMode(RangingMode.Short, 100);
        RobotContainer.armTab.addBoolean("Holding Game Piece", this::isHoldingGamePiece);

        RobotContainer.armTab.addDouble("Gamepiece distance", TOF::getRange);
    }

    public void setRollerState(State rollerState) {
        this.rollerState = rollerState;
    }

    public double getGamePieceOffset() {
        if(!TOF.isRangeValid() || gamePiece == GamePiece.Cube) {
            return 0;
        }
        return (Constants.Rollers.INTAKE_WIDTH / 2) - (TOF.getRange()/1000 + Constants.Rollers.CONE_WIDTH / 2);
    }

    public void setGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    public double getRollerSpeed() {
        return gamePiece == GamePiece.Cone ? rollerState.coneSpeed : rollerState.cubeSpeed;
    }

    public boolean isHoldingGamePiece() {
        return TOF.getRange()/1000 < Constants.Rollers.INTAKE_WIDTH  - 0.05 || Robot.isSimulation();
    }

    public void periodic() {
        motor.set(getRollerSpeed());
        if (getRollerSpeed() != 0) {
            holdingGamePiece = Math.abs(motor.getEncoder().getVelocity()) < Constants.Rollers.HOLDING_PIECE_VELOCITY;
        }

        SmartDashboard.putNumber("Gamepiece Sensor", getGamePieceOffset());
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
