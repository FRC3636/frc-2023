package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;
import frc.robot.RobotContainer;

import static frc.robot.Constants.Arm.*;


public class Arm extends SubsystemBase {
    private final CANSparkMax shoulder1 = new CANSparkMax(SHOULDER_1_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax shoulder2 = new CANSparkMax(SHOULDER_2_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final AbsoluteEncoder shoulderEncoder = shoulder1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private final ArmFeedforward shoulderFeedforward = new ArmFeedforward(Constants.Arm.SHOULDER_KS, Constants.Arm.SHOULDER_KG,
            Constants.Arm.SHOULDER_KV, Constants.Arm.SHOULDER_KA);
    private final ShoulderPosition shoulderTarget = ShoulderPosition.Stowed;

    private final PIDController wristPID = new PIDController(0, 0, 0);

    private final CANSparkMax wrist = new CANSparkMax(WRIST_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DigitalInput wristLimitSwitch = new DigitalInput(WRIST_LIMIT_SWITCH);


    public Arm() {
        shoulder1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shoulder2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shoulder1.setSmartCurrentLimit(40);
        shoulder2.setSmartCurrentLimit(40);
        shoulder2.follow(shoulder1, true);
        shoulderEncoder.setPositionConversionFactor(Units.rotationsToDegrees(SHOULDER_GEAR_RATIO));
        shoulderEncoder.setInverted(true);

        wrist.setIdleMode(CANSparkMax.IdleMode.kBrake);
        wrist.getEncoder().setPositionConversionFactor(Units.rotationsToDegrees(WRIST_GEAR_RATIO));
        wrist.setInverted(true);

        RobotContainer.armTab.add("Wrist PID", wristPID).withWidget(BuiltInWidgets.kPIDController);
    }

    public void driveShoulder(double speed) {
        shoulder1.set(speed);
    }

    public void updateWrist() {
        wrist.set(wristPID.calculate(wrist.getEncoder().getPosition(), shoulderEncoder.getPosition()));
    }

    public void driveWrist(double speed) {
        wrist.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Angle", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Shoulder Set Point", shoulderTarget.position);

        shoulder1.set(shoulderFeedforward.calculate(Units.degreesToRadians(shoulderTarget.position), 0));

        SmartDashboard.putBoolean("Wrist Limit Switch", wristLimitSwitch.get());
        SmartDashboard.putNumber("Wrist Angle", wrist.getEncoder().getPosition());

        if(!wristLimitSwitch.get()) {
            wrist.getEncoder().setPosition(WRIST_LIMIT_SWITCH_OFFSET);
        }
    }

    enum ShoulderPosition {
        High(ARM_HIGH_ANGLE),
        Mid(ARM_MID_ANGLE),
        Low(ARM_LOW_ANGLE),
        Stowed(ARM_STOWED_ANGLE);
        private final double position;

        ShoulderPosition(double position) {
            this.position = position;
        }
    }
}
