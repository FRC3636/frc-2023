package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;
import frc.robot.utils.GamePiece;

public class Rollers {
    private Rollers.State rollerState = Rollers.State.Off;
    private GamePiece gamePiece = GamePiece.Cone;

    private final CANSparkMax motor = new CANSparkMax(Constants.Rollers.ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    public Rollers() {
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.restoreFactoryDefaults();
        //RobotContainer.armTab.addNumber("Roller Bus Voltage", motor::getBusVoltage).withWidget(BuiltInWidgets.kGraph);
        //RobotContainer.armTab.addNumber("Roller Applied Voltage", motor::getAppliedOutput).withWidget(BuiltInWidgets.kGraph);
        //RobotContainer.armTab.addNumber("Roller Compensation Voltage", motor::getVoltageCompensationNominalVoltage).withWidget(BuiltInWidgets.kGraph);
    }

    public void setRollerState(State rollerState) {
        this.rollerState = rollerState;
    }

    public void setGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    public double getRollerSpeed() {
        return gamePiece == GamePiece.Cone ? rollerState.coneSpeed : rollerState.cubeSpeed;
    }

    public void periodic() {
        motor.set(getRollerSpeed());
    }

    public boolean isHoldingGamePiece() {
        if (motor.getBusVoltage() < Constants.Rollers.HOLDING_PIECE_VOLTAGE) {
            return true;
            //motor.getAppliedOutput();
            //motor.getVoltageCompensationNominalVoltage();
        }
        return false;
    }

    public enum State {
        Intake(Constants.Rollers.INTAKE_CONE, Constants.Rollers.INTAKE_CUBE),
        Outtake(Constants.Rollers.OUTTAKE_CONE, Constants.Rollers.OUTTAKE_CUBE),
        Off(0, 0);

        public final double coneSpeed;
        public final double cubeSpeed;

        State(double coneSpeed, double cubeSpeed) {
            this.coneSpeed = coneSpeed;
            this.cubeSpeed = cubeSpeed;
        }

    }
}
