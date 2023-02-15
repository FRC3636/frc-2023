package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Claw extends SubsystemBase {

    private final CANSparkMax claw = new CANSparkMax(Constants.Claw.CLAW_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DigitalInput clawLimitSwitch = new DigitalInput(Constants.Claw.CLAW_LIMIT_SWITCH);
    private ClawPosition clawPosition = ClawPosition.Cube;

    private final CANSparkMax rollers = new CANSparkMax(Constants.Claw.ROLLERS_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);

    private final PIDController clawPID = new PIDController(2, 0.1, 0);

    public Claw() {

        RobotContainer.armTab.add("Claw PID", clawPID).withWidget(BuiltInWidgets.kPIDController);
        claw.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rollers.setIdleMode(CANSparkMax.IdleMode.kBrake);

        claw.setInverted(false);
        claw.getEncoder().setPositionConversionFactor(Units.rotationsToRadians(Constants.Claw.CLAW_GEAR_RATIO));
        claw.getEncoder().setPosition(ClawPosition.Closed.position);
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Claw Limit Switch", clawLimitSwitch.get());

        boolean inThreshold = Math.abs(Units.rotationsToRadians(claw.getEncoder().getPosition()) - ClawPosition.Open.position) < Constants.Claw.CLAW_ANGLE_THRESHOLD;
        double targetAngleDifference = Units.rotationsToRadians(claw.getEncoder().getPosition()) - clawPosition.position;

        SmartDashboard.putNumber("set angle", targetAngleDifference);
        SmartDashboard.putNumber("Claw Angle", Units.rotationsToRadians(claw.getEncoder().getPosition()));
        SmartDashboard.putBoolean("In Threshold", inThreshold);
        



        if(clawPosition == ClawPosition.Open  && !inThreshold){
            if(clawLimitSwitch.get()){
                claw.set(-Constants.Claw.CLAW_SPEED);
            }else{
                claw.getEncoder().setPosition(0);
            }
        }else{
        
            if(!clawLimitSwitch.get()){
                claw.getEncoder().setPosition(0);
            }
        claw.set(clawPID.calculate(claw.getEncoder().getPosition(), clawPosition.position));
        }

    }


    public void setClawPosition(ClawPosition position) {
        this.clawPosition = position;
    }

    public void runRollers(int direction) {
        rollers.set(Constants.Claw.ROLLER_SPEED * direction);
    }

    public enum ClawPosition {
        Cone(Constants.Claw.CLAW_CONE_ANGLE),
        Cube(Constants.Claw.CLAW_CUBE_ANGLE),
        Open(0.0),
        Closed(Constants.Claw.CLAW_CLOSED_ANGLE);

        private final double position;

        ClawPosition(double position) {
            this.position = position;
        }
    }
}
