package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;

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
    private final Arm arm;

    private final CANSparkMax motor = new CANSparkMax(Constants.Wrist.ID, CANSparkMax.MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(Constants.Wrist.LIMIT_SWITCH);

    private final PIDController pidController = new PIDController(Constants.Wrist.KP, Constants.Wrist.KI, Constants.Wrist.KD);
    private final ArmFeedforward feedforward = new ArmFeedforward(Constants.Wrist.KS, Constants.Wrist.KG, Constants.Wrist.KV, Constants.Wrist.KA);

    public Wrist(Arm arm) {
        this.arm = arm;

        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.getEncoder().setPositionConversionFactor(Units.rotationsToRadians(1) * Constants.Wrist.GEAR_RATIO);
        motor.setSmartCurrentLimit(40);
        motor.setInverted(false);

        RobotContainer.armTab.add("Wrist PID", pidController).withWidget(BuiltInWidgets.kPIDController);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(motor.getEncoder().getPosition());
    }

    public void followShoulder() {
        followShoulderWithVelocity(arm.getShoulderVelocity());
    }

    public void followShoulderWithVelocity(Rotation2d velocity) {
        if(Arm.State.getTarget() == Arm.State.Stowed && Arm.State.getRollerSpeed() == 0) {
            velocity = Rotation2d.fromRadians(velocity.getRadians() + 1);
        }
        runWithSetpoint(getSetPosition(), velocity);
    }

    public void runWithVelocity(Rotation2d velocity) {
        runWithSetpoint(Rotation2d.fromRadians(motor.getEncoder().getPosition()), velocity);
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

    public double getMinAngle(double height){
        double intakeLength = Constants.Wrist.JOINT_TO_CORNER_DISTANCE;
        double intakeAngleOffset = Constants.Wrist.HORIZONTAL_TO_CORNER_ANGLE;
        double clearance = Constants.Wrist.CLEARANCE_HEIGHT;
        double angle = -Math.asin((height-clearance)/intakeLength)+intakeAngleOffset;
//        System.out.println("Math vs Real angle diff(degrees)=" + ((angle-motor.getEncoder().getPosition()))*(360/2/Math.PI));
        return angle;
    }

    public Rotation2d getSetPosition() {
        if(
                arm.getShoulderAngle().getRadians() < Constants.Wrist.MIN_SHOULDER_ANGLE.getRadians()
                        || Arm.State.getTarget().getShoulderAngle().getRadians() < Constants.Wrist.MIN_SHOULDER_ANGLE.getRadians()) {
            return Rotation2d.fromRadians(Math.max(0, Arm.State.getTarget().getWristAngle().getRadians()));
        }

        return Arm.State.getTarget().getWristAngle();
    }

    public double safeHeight(double armAngle) {
        double armHeight = Constants.Arm.PIVOT_HEIGHT;
        double armLength = Constants.Arm.HUMERUS_LENGTH;
        double height = armHeight - Math.cos(armAngle) * armLength;
//        System.out.println("Joint To Ground Height(in)-----> " + height);
        return height;
    }


    public boolean isLimitSwitchPressed() {
        return !limitSwitch.get();
    }

    public void periodic() {
        SmartDashboard.putBoolean("Wrist Limit Switch", limitSwitch.get());
        SmartDashboard.putNumber("Wrist Angle", Units.radiansToDegrees(motor.getEncoder().getPosition()));
        SmartDashboard.putNumber("Wrist Set Point", Arm.State.getTarget().getWristAngle().getDegrees());
        SmartDashboard.putNumber("Wrist Relative", arm.getWristAngle().getDegrees());
        SmartDashboard.putNumber("minSafeAngle", (360.0/2.0/Math.PI)*getMinAngle(safeHeight(arm.getShoulderAngle().getDegrees())));

        if(isLimitSwitchPressed()) {
            motor.getEncoder().setPosition(Constants.Wrist.LIMIT_SWITCH_OFFSET.getRadians());
        }
    }
}
