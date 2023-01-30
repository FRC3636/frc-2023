package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final CANSparkMax shoulder = new CANSparkMax(Constants.Arm.SHOULDER_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax claw = new CANSparkMax(Constants.Arm.CLAW_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rollers = new CANSparkMax(Constants.Arm.ROLLERS_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final AnalogPotentiometer potentiometer = new AnalogPotentiometer(
            Constants.Arm.POTENTIOMETER_PORT,
            Constants.Arm.POTENTIOMETER_RANGE,
            Constants.Arm.POTENTIOMETER_OFFSET
    );

    public Arm() {

        shoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shoulder.getEncoder().setPositionConversionFactor(Constants.Arm.SHOULDER_GEAR_RATIO);
        shoulder.setSmartCurrentLimit(40);
    }

    public void driveShoulder(double speed) {
        shoulder.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder",shoulder.getEncoder().getPosition());
        SmartDashboard.putNumber("Potentiometer", potentiometer.get());
    }

    enum ShoulderPosition {
        High(Constants.Arm.ARM_HIGH_ANGLE),
        Mid(Constants.Arm.ARM_MID_ANGLE),
        Low(Constants.Arm.ARM_LOW_ANGLE),
        Stored(0);
        private final double position;

        ShoulderPosition(double position) {
            this.position = position;
        }
    }
}
