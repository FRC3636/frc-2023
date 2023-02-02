package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final CANSparkMax shoulder = new CANSparkMax(Constants.Arm.SHOULDER_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax claw = new CANSparkMax(Constants.Arm.CLAW_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rollers = new CANSparkMax(Constants.Arm.ROLLERS_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DigitalInput clawLimitSwitch = new DigitalInput(Constants.Arm.CLAW_LIMITSWITCH);

    private final AnalogPotentiometer potentiometer = new AnalogPotentiometer(
            Constants.Arm.POTENTIOMETER_PORT,
            Constants.Arm.POTENTIOMETER_RANGE,
            Constants.Arm.POTENTIOMETER_OFFSET);

    private ClawPosition clawPosition = ClawPosition.Open;

    public Arm() {
        shoulder.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shoulder.getEncoder().setPositionConversionFactor(Constants.Arm.SHOULDER_GEAR_RATIO);
        shoulder.setSmartCurrentLimit(40);
    }

    public void driveShoulder(double speed) {
        shoulder.set(speed);
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Claw Limit Switch", clawLimitSwitch.get());
        SmartDashboard.putNumber("Shoulder", shoulder.getEncoder().getPosition());
        SmartDashboard.putNumber("Potentiometer", potentiometer.get());
        double targetAngleDifference = Units.rotationsToRadians(claw.getEncoder().getPosition()) - clawPosition.position;

        switch (clawPosition) {
            case Open:
                if (clawLimitSwitch.get()) {
                    claw.set(0);
                }else {
                    
                    claw.getEncoder().setPosition(0);
                    claw.set(Constants.Arm.CLAW_SPEED);
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

    public void setIntakeSpeed(int speed) {
        rollers.set(speed);
    }

    public void runRollers(int direction){
        rollers.set(Constants.Arm.ROLLER_SPEED*direction);
    }

    enum ShoulderPosition {
        High(Constants.Arm.ARM_HIGH_ANGLE),
        Mid(Constants.Arm.ARM_MID_ANGLE),
        Low(Constants.Arm.ARM_LOW_ANGLE),
        Stored(0);

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

        ClawPosition(double position){
            this.position = position;
        }
    }
}
