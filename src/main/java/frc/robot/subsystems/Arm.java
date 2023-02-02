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
    private final CANSparkMax claw = new CANSparkMax(Constants.Arm.CLAW_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rollers = new CANSparkMax(Constants.Arm.ROLLERS_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DigitalInput clawLimitSwitch = new DigitalInput(Constants.Arm.CLAW_LIMIT_SWITCH);

    private ClawPosition clawPosition = ClawPosition.Closed;


    public Arm() {
        shoulder1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shoulder2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shoulder1.setSmartCurrentLimit(40);
        shoulder2.setSmartCurrentLimit(40);

        shoulderEncoder.setPositionConversionFactor(Units.degreesToRotations(Constants.Arm.SHOULDER_GEAR_RATIO));

        shoulder2.follow(shoulder1, true);
    }

    public void driveShoulder(double speed) {
        shoulder1.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Angle", shoulderEncoder.getPosition());
        SmartDashboard.putBoolean("Claw Limit Switch", clawLimitSwitch.get());

        double targetAngleDifference = claw.getEncoder().getPosition() - clawPosition.position;

        SmartDashboard.putNumber("set angle", targetAngleDifference);

        SmartDashboard.putBoolean("running", Math.abs(targetAngleDifference) > Constants.Arm.CLAW_CLAMP_THRESHOLD);

        switch (clawPosition) {
            case Open:
                if (clawLimitSwitch.get()) {
                    claw.set(0);
                    claw.getEncoder().setPosition(0);
                } else {
                    claw.set(-Constants.Arm.CLAW_SPEED);
                }
                break;
            default:
                if (Math.abs(targetAngleDifference) > Constants.Arm.CLAW_CLAMP_THRESHOLD) {
                    claw.set((targetAngleDifference > 0) ? Constants.Arm.CLAW_SPEED : -Constants.Arm.CLAW_SPEED);
                } else {
                    claw.set(0);
                }
                break;
        }

    }

    public void setClawPosition(ClawPosition position) {
        this.clawPosition = position;
    }

    public void runRollers(int direction) {
        rollers.set(Constants.Arm.ROLLER_SPEED * direction);
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

    public enum ClawPosition {
        Cone(Constants.Arm.CLAW_CONE_ANGLE),
        Cube(Constants.Arm.CLAW_CUBE_ANGLE),
        Open(0.0),
        Closed(Constants.Arm.CLAW_CLOSED_ANGLE);

        private final double position;

        ClawPosition(double position) {
            this.position = position;
        }
    }
}
