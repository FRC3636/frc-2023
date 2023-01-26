package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final CANSparkMax shoulder = new CANSparkMax(
            Constants.Arm.SHOULDER_MOTOR,
            CANSparkMaxLowLevel.MotorType.kBrushless
    );

    public Arm() {
        shoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shoulder.setSmartCurrentLimit(40);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
