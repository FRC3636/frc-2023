package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmState;
import frc.robot.Constants;

public class Rollers extends SubsystemBase {

    private final CANSparkMax rollers = new CANSparkMax(Constants.Rollers.ID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    public Rollers() {
        rollers.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void intake() {
        rollers.set((ArmState.getGamePiece() == ArmState.GamePiece.Cone ? Constants.Rollers.INTAKE_CONE : Constants.Rollers.INTAKE_CUBE) * Constants.Rollers.SPEED);
    }

    public void outtake() {
        rollers.set((ArmState.getGamePiece() == ArmState.GamePiece.Cone ? Constants.Rollers.INTAKE_CUBE : Constants.Rollers.INTAKE_CONE) * Constants.Rollers.SPEED);
    }
    public void stop() {
        rollers.set(0);
    }
}
