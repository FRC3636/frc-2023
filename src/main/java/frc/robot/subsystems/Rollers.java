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

public class Rollers extends SubsystemBase {

    private final CANSparkMax rollers = new CANSparkMax(Constants.Rollers.ROLLERS_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    public Rollers() {
        rollers.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void runRollers(RollerDirection direction) {
        rollers.set(direction.value * Constants.Rollers.ROLLER_SPEED);
    }

    public void stop() {
        rollers.set(0);
    }

    public enum RollerDirection {
        IntakeCone(Constants.Rollers.INTAKE_CONE),
        IntakeCube(Constants.Rollers.INTAKE_CUBE),
        OuttakeCone(Constants.Rollers.INTAKE_CONE),
        OuttakeCube(Constants.Rollers.INTAKE_CUBE),
        Off(0);

        public final int value;
        RollerDirection(int value) {
            this.value = value;
        }
    }
}
