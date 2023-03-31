package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Wrist {
    protected final Arm arm;

    private final CANSparkMax motor = new CANSparkMax(Constants.Wrist.ID, CANSparkMax.MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(Constants.Wrist.LIMIT_SWITCH);
    private final AbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private Rotation2d wristAngleOffset = new Rotation2d();

    protected final PIDController pidController = new PIDController(Constants.Wrist.KP, Constants.Wrist.KI, Constants.Wrist.KD);
    protected final ArmFeedforward feedforward = new ArmFeedforward(Constants.Wrist.KS, Constants.Wrist.KG, Constants.Wrist.KV, Constants.Wrist.KA);


    public Wrist(Arm arm) {
        this.arm = arm;

        motor.restoreFactoryDefaults();

        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.getEncoder().setPositionConversionFactor(Units.rotationsToRadians(1) * Constants.Wrist.GEAR_RATIO);
        motor.getEncoder().setVelocityConversionFactor(Units.rotationsToRadians(1) * Constants.Wrist.GEAR_RATIO / 60.0);
        encoder.setPositionConversionFactor(Units.rotationsToRadians(1));
        encoder.setVelocityConversionFactor(Units.rotationsToRadians(1) / 60.0);
        motor.getEncoder().setMeasurementPeriod(8);
        motor.setSmartCurrentLimit(40);
        motor.setInverted(true);
        encoder.setInverted(false);

        zeroEncoder();

        RobotContainer.armTab.add("Wrist PID", pidController).withWidget(BuiltInWidgets.kPIDController);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(motor.getEncoder().getPosition()).plus(wristAngleOffset);
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRadians(encoder.getPosition() < Constants.Wrist.ABSOLUTE_ENCODER_WRAPPING_ANGLE.getRadians() ? encoder.getPosition() : encoder.getPosition() - Math.PI * 2);
    }

    public void followShoulderWithVelocity(Rotation2d velocity) {
        if(arm.getTarget() == Arm.State.Stowed && arm.getRollerState() == Rollers.State.Off) {
            velocity = Rotation2d.fromRadians(velocity.getRadians() + 1);
        }
        Rotation2d position = arm.getTargetWristAngle();
        if(velocity.getRadians() != 0) {
            Rotation2d minAngle = getWristAngleFromHeight(0.4, arm.getShoulderAngle());
            position = Rotation2d.fromRadians(Math.max(position.getRadians(), minAngle.getRadians()));
        }
        runWithSetpoint(position, velocity);
    }

    public void followShoulder() {
        runWithSetpoint(arm.getTargetWristAngle(), arm.getShoulderVelocity().unaryMinus());
    }

    public void runWithVelocity(Rotation2d velocity) {
        runWithSetpoint(Rotation2d.fromRadians(encoder.getPosition()), velocity);
    }

    public void runWithSetpoint(Rotation2d position, Rotation2d velocity) {
        velocity = Rotation2d.fromRadians(velocity.getRadians() + pidController.calculate(arm.getWristAngle().getRadians(), position.getRadians()));

        SmartDashboard.putNumber("Wrist Setpoint", position.getRadians());

        if (isLimitSwitchPressed() && velocity.getRadians() >= 0) {
            motor.set(0);
            return;
        }

        motor.setVoltage(feedforward.calculate(arm.getWristAngle().getRadians(), velocity.getRadians()));
    }

    public static Rotation2d getWristAngleFromHeight(double height, Rotation2d shoulderAngle) {
        double jointHeight = Constants.Arm.PIVOT_HEIGHT - Constants.Arm.HUMERUS_LENGTH * shoulderAngle.getCos();
        double sinAngle = Math.max(Math.min((height - jointHeight) / Constants.Arm.MANIPULATOR_LENGTH, 1), -1);
        return Rotation2d.fromRadians(Math.asin(sinAngle));
    }

    public boolean isLimitSwitchPressed() {
        return !limitSwitch.get();
    }

    public void periodic() {
        SmartDashboard.putBoolean("Wrist Limit Switch", limitSwitch.get());
        SmartDashboard.putNumber("Wrist Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Wrist Set Point", arm.getTargetWristAngle().getDegrees());
        SmartDashboard.putNumber("Wrist Target Height", arm.getTargetHeight());
        SmartDashboard.putNumber("Wrist Relative", arm.getWristAngle().getDegrees());

        adjustWristAngle();
    }

    public void adjustWristAngle() {
        if(encoder.getPosition() != 0) {
            wristAngleOffset = wristAngleOffset.plus(
                    Rotation2d.fromRadians(
                            Math.signum(getAbsoluteAngle().minus(getAngle()).getRadians()) *
                                    Constants.Wrist.ENCODER_INTERPOLATION_SPEED.getRadians()
                    )
            );
        }
    }

    public void setIdleMode(CANSparkMax.IdleMode idleMode) {
        motor.setIdleMode(idleMode);
    }

    private void zeroEncoder() {
        wristAngleOffset = new Rotation2d();
        motor.getEncoder().setPosition(getAbsoluteAngle().getRadians());
    }
}
