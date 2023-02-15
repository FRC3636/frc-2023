package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import org.opencv.core.Mat;

public class Shoulder extends SubsystemBase {
    private final CANSparkMax motor1 = new CANSparkMax(Constants.Shoulder.SHOULDER_1_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax motor2 = new CANSparkMax(Constants.Shoulder.SHOULDER_2_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final AbsoluteEncoder encoder = motor1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private ArmFeedforward feedforwardController = new ArmFeedforward(Constants.Shoulder.SHOULDER_KS, Constants.Shoulder.SHOULDER_KG,
            Constants.Shoulder.SHOULDER_KV, Constants.Shoulder.SHOULDER_KA);
    private final PIDController pidController = new PIDController(Constants.Shoulder.SHOULDER_KP, Constants.Shoulder.SHOULDER_KI,
            Constants.Shoulder.SHOULDER_KD);

//    private TrapezoidProfile trapezoidProfile = new TrapezoidProfile();

//    private TrapezoidProfileCommand

    public Shoulder() {
        motor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor1.setSmartCurrentLimit(40);
        motor2.setSmartCurrentLimit(40);
        motor2.follow(motor1, true);
        encoder.setPositionConversionFactor(Units.rotationsToRadians(1) * Constants.Shoulder.SHOULDER_GEAR_RATIO);

        // RobotContainer.armTab.add("Shoulder PID", pidController).withWidget(BuiltInWidgets.kPIDController);

        motor1.setInverted(true);
        encoder.setInverted(true);
    }

    public double getActualPosition() {
        return encoder.getPosition() 
            > ((2*Math.PI) * Constants.Shoulder.SHOULDER_GEAR_RATIO - Math.PI / 8) 
                ? encoder.getPosition() - ((2*Math.PI) * Constants.Shoulder.SHOULDER_GEAR_RATIO)
                : encoder.getPosition();
    }

    public double getActualVelocity() {
        return encoder.getVelocity() * Constants.Shoulder.SHOULDER_GEAR_RATIO;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Angle", getActualPosition());
        SmartDashboard.putNumber("Shoulder Angle Measured", encoder.getPosition());

    }

    /// Run the shoulder at the given setpoints using the feedforward and feedback controllers.
    /// @param position The position setpoint. Measured in radians from the vertical.
    /// @param velocity The velocity setpoint. Measured in radians per second.
    /// @param acceleration The acceleration setpoint. Measured in radians per second squared.
    public void runWithSetpoint(double position, double velocity, double acceleration) {
        velocity += pidController.calculate(signedModularDistance(getActualPosition(), position, Math.PI * 2), 0);

        double voltage = feedforwardController.calculate(position - Math.PI / 2, velocity, acceleration);

        motor1.setVoltage(voltage);

        SmartDashboard.putNumber("Shoulder Applied Voltage", voltage);
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
