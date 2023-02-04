package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
    private final ArmFeedforward shoulderFeedforward = new ArmFeedforward(Constants.Arm.SHOULDER_KS, Constants.Arm.SHOULDER_KG,
            Constants.Arm.SHOULDER_KV, Constants.Arm.SHOULDER_KA);
    private final PIDController shoulderPID = new PIDController(Constants.Arm.SHOULDER_KP, Constants.Arm.SHOULDER_KI,
            Constants.Arm.SHOULDER_KD);
    private ShoulderPosition shoulderTarget = ShoulderPosition.Stowed;

    private final CANSparkMax wrist = new CANSparkMax(Constants.Arm.WRIST_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DigitalInput wristLimitSwitch = new DigitalInput(Constants.Arm.WRIST_LIMIT_SWITCH);


    public Arm() {
        shoulder1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shoulder2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shoulder1.setSmartCurrentLimit(40);
        shoulder2.setSmartCurrentLimit(40);
        shoulder2.follow(shoulder1, true);
        shoulderEncoder.setPositionConversionFactor(Units.rotationsToDegrees(1) * Constants.Arm.SHOULDER_GEAR_RATIO);

        wrist.setIdleMode(CANSparkMax.IdleMode.kBrake);
        wrist.getEncoder().setPositionConversionFactor(Units.rotationsToDegrees(Constants.Arm.SHOULDER_GEAR_RATIO));
    }

    public void setShoulderPosition(ShoulderPosition pos) {
        shoulderTarget = pos;
    }

    public void driveWrist(double speed) {
        wrist.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Angle", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Shoulder Set Point", shoulderTarget.position);

        shoulder1.set(
            shoulderFeedforward.calculate(Units.degreesToRadians(shoulderTarget.position), 0)
            + shoulderPID.calculate(Units.degreesToRadians(shoulderEncoder.getPosition()), Units.degreesToRadians(shoulderTarget.position))
        );

        SmartDashboard.putBoolean("Wrist Limit Switch", wristLimitSwitch.get());
        SmartDashboard.putNumber("Wrist Angle", wrist.getEncoder().getPosition());
    }

    enum ShoulderPosition {
        High(Constants.Arm.SHOULDER_HIGH_ANGLE),
        Mid(Constants.Arm.SHOULDER_MID_ANGLE),
        Low(Constants.Arm.SHOULDER_LOW_ANGLE),
        Stowed(Constants.Arm.SHOULDER_STOWED_ANGLE);
        private final double position;

        ShoulderPosition(double position) {
            this.position = position;
        }
    }
}
