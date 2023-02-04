package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(Constants.Wrist.WRIST_ID, CANSparkMax.MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(Constants.Wrist.WRIST_LIMIT_SWITCH);

    private PIDController pidController = new PIDController(Constants.Wrist.WRIST_KP, Constants.Wrist.WRIST_KI, Constants.Wrist.WRIST_KD);

    private Position targetPosition = Position.Horizontal;

    public Wrist() {
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.getEncoder().setPositionConversionFactor(Units.rotationsToRadians(1) * Constants.Wrist.WRIST_GEAR_RATIO);
    }

    public void setTargetPosition(Position pos) {
        targetPosition = pos;
    }

    public double getAngleToFrame() {
        return RobotContainer.shoulder.getActualPosition() + motor.getEncoder().getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Wrist Limit Switch", limitSwitch.get());
        SmartDashboard.putNumber("Wrist Angle", motor.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Set Point", targetPosition.position);

        if (limitSwitch.get()) {
            motor.getEncoder().setPosition(Constants.Wrist.WRIST_LIMIT_SWITCH_OFFSET);
        }

        motor.set(pidController.calculate(getAngleToFrame(), targetPosition.position));
    }

    public enum Position {
        Horizontal(0),
        Vertical(-Math.PI / 2);

        private final double position;

        private Position(double position) {
            this.position = position;
        }
    }
}
