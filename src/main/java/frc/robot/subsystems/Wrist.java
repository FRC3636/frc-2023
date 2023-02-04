package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
    private final CANSparkMax wrist = new CANSparkMax(Constants.Arm.WRIST_ID, CANSparkMax.MotorType.kBrushless);
    private final DigitalInput wristLimitSwitch = new DigitalInput(Constants.Arm.WRIST_LIMIT_SWITCH);

    public Wrist() {
        wrist.setIdleMode(CANSparkMax.IdleMode.kBrake);
        wrist.getEncoder().setPositionConversionFactor(Units.rotationsToRadians(1) * Constants.Arm.WRIST_GEAR_RATIO);
    }

    public void drive(double speed) {
        wrist.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Wrist Limit Switch", wristLimitSwitch.get());
        SmartDashboard.putNumber("Wrist Angle", wrist.getEncoder().getPosition());
    }
}
