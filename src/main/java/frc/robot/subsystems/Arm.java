package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    CANSparkMax shoulder = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);

    public Arm( ) {
        shoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shoulder.setSmartCurrentLimit(40);
    }

    public void driveShoulder(double speed) {
        shoulder.set(speed);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
