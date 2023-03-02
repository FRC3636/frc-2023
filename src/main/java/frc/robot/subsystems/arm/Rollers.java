package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;

public class Rollers {

    private final CANSparkMax motor = new CANSparkMax(Constants.Rollers.ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public Rollers() {
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.restoreFactoryDefaults();
    }

    public void periodic() {
        motor.set(Arm.State.getRollerSpeed());
    }

    public enum State {
        Intake(Constants.Rollers.INTAKE_CONE, Constants.Rollers.INTAKE_CUBE),
        Outtake(Constants.Rollers.OUTTAKE_CONE, Constants.Rollers.OUTTAKE_CUBE),
        Off(0, 0);

        public final double coneSpeed;
        public final double cubeSpeed;

        State(double coneSpeed, double cubeSpeed) {
            this.coneSpeed = coneSpeed;
            this.cubeSpeed = cubeSpeed;
        }

    }
}
