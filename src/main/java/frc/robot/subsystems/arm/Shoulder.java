package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.GamePiece;

public class Shoulder {
    protected GamePiece gamePiece;

    private final CANSparkMax motor1 = new CANSparkMax(Constants.Shoulder.MOTOR_1_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);

    private final CANSparkMax motor2 = new CANSparkMax(Constants.Shoulder.MOTOR_2_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final AbsoluteEncoder encoder = motor1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    protected ArmFeedforward feedforwardController = new ArmFeedforward(Constants.Shoulder.KS, Constants.Shoulder.KG,
            Constants.Shoulder.KV, Constants.Shoulder.KA);

    protected final PIDController dynamicPIDController = new PIDController(Constants.Shoulder.DYNAMIC_KP, Constants.Shoulder.KI,
            Constants.Shoulder.KD);
    protected final PIDController staticPIDController = new PIDController(Constants.Shoulder.STATIC_KP, Constants.Shoulder.KI,
            Constants.Shoulder.KD);

    public Shoulder() {
        motor1.restoreFactoryDefaults();
        motor2.restoreFactoryDefaults();

        motor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor1.setSmartCurrentLimit(40);
        motor2.setSmartCurrentLimit(40);
        motor2.follow(motor1, true);
        motor1.getEncoder().setPositionConversionFactor(Units.rotationsToRadians(1) / 151.2);
        encoder.setPositionConversionFactor(Units.rotationsToRadians(1));
        encoder.setVelocityConversionFactor(Units.rotationsToRadians(1) / 60.0);

        RobotContainer.armTab.add("Shoulder PID", dynamicPIDController).withWidget(BuiltInWidgets.kPIDController);

        motor1.setInverted(true);
        encoder.setInverted(true);

        dynamicPIDController.setTolerance(Units.degreesToRadians(1));
        motor1.getEncoder().setPosition(0);

        initialize();
    }

    public void setIdleMode(CANSparkMax.IdleMode idleMode) {
        motor1.setIdleMode(idleMode);
        motor2.setIdleMode(idleMode);
    }

    public static Rotation2d getShoulderAngleFromHeight(double height, Rotation2d wristAngle) {
        double jointHeight = height - Constants.Arm.MANIPULATOR_LENGTH * wristAngle.getSin();
        double cosAngle = Math.max(Math.min((Constants.Arm.PIVOT_HEIGHT - jointHeight) / Constants.Arm.HUMERUS_LENGTH, 1), -1);
        return Rotation2d.fromRadians(Math.acos(cosAngle));
    }

    public void setGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    public void initialize() {
        motor1.getEncoder().setPosition(getAngle().getRadians());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(
                encoder.getPosition() > Constants.Shoulder.MAX_ANGLE.getRadians()
                ? encoder.getPosition() - Math.PI * 2
                : encoder.getPosition());
    }

    public Rotation2d getVelocity() {
        return Rotation2d.fromRadians(encoder.getVelocity());
    }


    public void periodic() {
        SmartDashboard.putNumber("Shoulder Angle", getAngle().getRadians());
        SmartDashboard.putNumber("Shoulder Velocity", getVelocity().getRadians());
    }

    /// Run the shoulder at the given setpoints using the feedforward and feedback controllers.
    /// @param position The position setpoint. Measured in radians from the vertical.
    /// @param velocity The velocity setpoint. Measured in radians per second.
    /// @param acceleration The acceleration setpoint. Measured in radians per second squared.
    public void runWithSetpoint(Rotation2d position, Rotation2d velocity, Rotation2d acceleration) {
        PIDController pidController = velocity.equals(new Rotation2d()) ? this.staticPIDController : this.dynamicPIDController;
        velocity = Rotation2d.fromRadians(velocity.getRadians() +
                pidController.calculate(getAngle().getRadians(),
                        Math.max(
                                position.getRadians(),
                                Constants.Shoulder.STOWED_ANGLE.getRadians()
                        )
                )
        );

        double voltage = feedforwardController.calculate(
                getAngle().getRadians() - Math.PI / 2,
                velocity.getRadians(),
                acceleration.getRadians()
        );

        motor1.setVoltage(voltage);
    }

    static double signedModularDistance(double a, double b, double modulus) {
        a = (a % modulus + a) % modulus;
        b = (b % modulus + b) % modulus;

        double posDist, negDist;
        if (b >= a) {
            posDist = b - a;
            negDist = a + (modulus - b);
        } else {
            posDist = b + (modulus - a);
            negDist = a - b;
        }

        if (posDist > negDist) {
            return posDist;
        } else {
            return -negDist;
        }
    }
}
