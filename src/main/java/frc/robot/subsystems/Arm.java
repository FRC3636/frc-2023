package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final CANSparkMax shoulder1 = new CANSparkMax(Constants.Arm.SHOULDER_1_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax shoulder2 = new CANSparkMax(Constants.Arm.SHOULDER_2_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final AbsoluteEncoder shoulderEncoder = shoulder1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private final CANSparkMax wrist = new CANSparkMax(Constants.Arm.WRIST_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DigitalInput wristLimitSwitch = new DigitalInput(Constants.Arm.WRIST_LIMIT_SWITCH);


    public Arm() {
        shoulder1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shoulder2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shoulder1.setSmartCurrentLimit(40);
        shoulder2.setSmartCurrentLimit(40);
        shoulder2.follow(shoulder1, true);
        shoulderEncoder.setPositionConversionFactor(Units.rotationsToDegrees(Constants.Arm.SHOULDER_GEAR_RATIO));

        wrist.setIdleMode(CANSparkMax.IdleMode.kBrake);
        wrist.getEncoder().setPositionConversionFactor(Units.rotationsToDegrees(Constants.Arm.SHOULDER_GEAR_RATIO));
    }

    public void driveShoulder(double speed) {
        shoulder1.set(speed);
    }

    public void driveWrist(double speed) {
        wrist.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Angle", shoulderEncoder.getPosition());

        SmartDashboard.putBoolean("Wrist Limit Switch", wristLimitSwitch.get());
        SmartDashboard.putNumber("Wrist Angle", wrist.getEncoder().getPosition());
    }

    enum ShoulderPosition {
        High(Constants.Arm.ARM_HIGH_ANGLE),
        Mid(Constants.Arm.ARM_MID_ANGLE),
        Low(Constants.Arm.ARM_LOW_ANGLE),
        Stowed(Constants.Arm.ARM_STOWED_ANGLE);
        private final double position;

        ShoulderPosition(double position) {
            this.position = position;
        }
    }
}
