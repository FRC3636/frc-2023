package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

    private final CANSparkMax claw = new CANSparkMax(Constants.Arm.CLAW_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DigitalInput clawLimitSwitch = new DigitalInput(Constants.Arm.CLAW_LIMIT_SWITCH);
    private ClawPosition clawPosition = ClawPosition.Closed;

    private final CANSparkMax rollers = new CANSparkMax(Constants.Arm.ROLLERS_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);

    public Claw() {
        claw.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rollers.setIdleMode(CANSparkMax.IdleMode.kBrake);

        claw.getEncoder().setPositionConversionFactor(Units.rotationsToDegrees(Constants.Arm.CLAW_GEAR_RATIO));
        claw.getEncoder().setPosition(clawPosition.position);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Claw Limit Switch", clawLimitSwitch.get());

        double targetAngleDifference = claw.getEncoder().getPosition() - clawPosition.position;

        SmartDashboard.putNumber("set angle", targetAngleDifference);
        SmartDashboard.putNumber("Claw Angle", claw.getEncoder().getPosition());
        SmartDashboard.putBoolean("running", Math.abs(targetAngleDifference) > Constants.Arm.CLAW_CLAMP_THRESHOLD);

        if (clawLimitSwitch.get()) {
            claw.getEncoder().setPosition(0);
        }

        if (Math.abs(targetAngleDifference) > Constants.Arm.CLAW_CLAMP_THRESHOLD) {
            claw.set((targetAngleDifference < 0) ? Constants.Arm.CLAW_SPEED : -Constants.Arm.CLAW_SPEED);
        } else {
            claw.set(0);
        }
    }

    public void setClawPosition(ClawPosition position) {
        this.clawPosition = position;
    }

    public void runRollers(int direction) {
        rollers.set(Constants.Arm.ROLLER_SPEED * direction);
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
