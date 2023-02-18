package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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

    private final PIDController pidController = new PIDController(Constants.Wrist.KP, Constants.Wrist.KI, Constants.Wrist.KD);
    private final ArmFeedforward feedforward = new ArmFeedforward(Constants.Wrist.KS, Constants.Wrist.KG, Constants.Wrist.KV, Constants.Wrist.KA);

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

    public void followShoulder() {
        followShoulderWithVelocity(-RobotContainer.shoulder.getActualVelocity());
    }

    public void followShoulderWithVelocity(double velocity) {
        runWithSetpoint(getSetPosition(), velocity);
    }

    public void RunWithVelocity(double velocity) {
        runWithSetpoint(this.motor.getEncoder().getPosition(), velocity);
    }

    public void runWithSetpoint(double position, double velocity) {
        velocity += pidController.calculate(getAngleToFrame(), position);

        if (isLimitSwitchPressed()) {
            motor.getEncoder().setPosition(Constants.Wrist.LIMIT_SWITCH_OFFSET);
            if(velocity >= 0) {
                motor.set(0);
            }
        }

        motor.setVoltage(feedforward.calculate(getAngleToFrame(), velocity));
    }

    public double getSetPosition() {
        if(RobotContainer.shoulder.getActualPosition() > Constants.Wrist.MIN_SHOULDER_ANGLE) {
            return ArmState.target.wristAngle;
        }

        return Math.max(0, ArmState.target.wristAngle);
    }

    public double getPosition() {
        return this.motor.getEncoder().getPosition();
    }

    public boolean isLimitSwitchPressed() {
        return !limitSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Wrist Limit Switch", limitSwitch.get());
        SmartDashboard.putNumber("Wrist Angle", motor.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Set Point", ArmState.target.wristAngle);
        SmartDashboard.putNumber("Wrist Relative", getAngleToFrame());
    }
}
