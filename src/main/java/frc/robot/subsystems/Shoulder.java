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
    private final CANSparkMax motor1 = new CANSparkMax(Constants.Arm.SHOULDER_1_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax motor2 = new CANSparkMax(Constants.Arm.SHOULDER_2_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final AbsoluteEncoder encoder = motor1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private final ArmFeedforward feedforwardController = new ArmFeedforward(Constants.Arm.SHOULDER_KS, Constants.Arm.SHOULDER_KG,
            Constants.Arm.SHOULDER_KV, Constants.Arm.SHOULDER_KA);
    private final PIDController pidController = new PIDController(Constants.Arm.SHOULDER_KP, Constants.Arm.SHOULDER_KI,
            Constants.Arm.SHOULDER_KD);

    private Position targetPosition = Position.Stowed;

    public Shoulder() {
        motor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor1.setSmartCurrentLimit(40);
        motor2.setSmartCurrentLimit(40);
        motor2.follow(motor1, true);
        encoder.setPositionConversionFactor(Units.rotationsToRadians(1) * Constants.Arm.SHOULDER_GEAR_RATIO);

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

        motor1.set(
            feedforwardController.calculate(targetPosition.position, 0)
            + pidController.calculate(encoder.getPosition(), targetPosition.position)
        );
    }

    enum Position {
        High(Constants.Arm.SHOULDER_HIGH_ANGLE),
        Mid(Constants.Arm.SHOULDER_MID_ANGLE),
        Low(Constants.Arm.SHOULDER_LOW_ANGLE),
        Stowed(Constants.Arm.SHOULDER_STOWED_ANGLE);

        private final double position;

        Position(double position) {
            this.position = position;
        }
    }
}
