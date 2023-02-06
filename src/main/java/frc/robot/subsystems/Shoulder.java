package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoulder extends SubsystemBase {
    private final CANSparkMax motor1 = new CANSparkMax(Constants.Shoulder.SHOULDER_1_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax motor2 = new CANSparkMax(Constants.Shoulder.SHOULDER_2_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final AbsoluteEncoder encoder = motor1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private final ArmFeedforward feedforwardController = new ArmFeedforward(Constants.Shoulder.SHOULDER_KS, Constants.Shoulder.SHOULDER_KG,
            Constants.Shoulder.SHOULDER_KV, Constants.Shoulder.SHOULDER_KA);
    private final PIDController pidController = new PIDController(Constants.Shoulder.SHOULDER_KP, Constants.Shoulder.SHOULDER_KI,
            Constants.Shoulder.SHOULDER_KD);

    private Position targetPosition = null;

    public Shoulder() {
        motor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor1.setSmartCurrentLimit(40);
        motor2.setSmartCurrentLimit(40);
        motor2.follow(motor1, true);
        encoder.setPositionConversionFactor(Units.rotationsToRadians(1) * Constants.Shoulder.SHOULDER_GEAR_RATIO);
    }

    public void setTargetPosition(Position pos) {
        targetPosition = pos;
    }

    public double getActualPosition() {
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Angle", encoder.getPosition());
        SmartDashboard.putNumber("Shoulder Set Point", targetPosition.position);

        if (targetPosition == null) {
            motor1.set(0);
            return;
        }

        motor1.set(
            feedforwardController.calculate(targetPosition.position + Math.PI / 2, 0)
            + pidController.calculate(signedModularDistance(targetPosition.position, encoder.getPosition(), 2 * Math.PI), 0)
        );
    }

    public enum Position {
        High(Constants.Shoulder.SHOULDER_HIGH_ANGLE),
        Mid(Constants.Shoulder.SHOULDER_MID_ANGLE),
        Low(Constants.Shoulder.SHOULDER_LOW_ANGLE),
        Stowed(Constants.Shoulder.SHOULDER_STOWED_ANGLE);

        private final double position;

        Position(double position) {
            this.position = position;
        }
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
