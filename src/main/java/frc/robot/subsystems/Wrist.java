package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmState;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(Constants.Wrist.ID, CANSparkMax.MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(Constants.Wrist.LIMIT_SWITCH);

    private PIDController pidController = new PIDController(Constants.Wrist.KP, Constants.Wrist.KI, Constants.Wrist.KD);
    private ArmFeedforward feedforward = new ArmFeedforward(Constants.Wrist.KS, Constants.Wrist.KG, Constants.Wrist.KV, Constants.Wrist.KA);

    public Wrist() {
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.getEncoder().setPositionConversionFactor(Units.rotationsToRadians(1) * Constants.Wrist.GEAR_RATIO);
        motor.setSmartCurrentLimit(40);
        motor.setInverted(false);

        RobotContainer.armTab.add("Wrist PID", pidController).withWidget(BuiltInWidgets.kPIDController);
    }

    public double getAngleToFrame() {
        return RobotContainer.shoulder.getActualPosition() + motor.getEncoder().getPosition();
    }

    //FIXME Don't switch to period until shoulder encoder doesn't wrap to 180
    public void temporaryUpdateWrist() {
        motor.setVoltage(
                feedforward.calculate(getAngleToFrame(), -RobotContainer.shoulder.getActualVelocity()) +
                pidController.calculate(getAngleToFrame(), ArmState.target.wristAngle)
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Wrist Limit Switch", limitSwitch.get());
        SmartDashboard.putNumber("Wrist Angle", motor.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Set Point", ArmState.target.wristAngle);
        SmartDashboard.putNumber("Wrist Relative", getAngleToFrame());

        if (!limitSwitch.get()) {
            motor.getEncoder().setPosition(Constants.Wrist.LIMIT_SWITCH_OFFSET);
            motor.set(0);
        }
    }
}
