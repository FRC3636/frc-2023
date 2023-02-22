package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ArmState;
import frc.robot.subsystems.Shoulder;

public class ShoulderHoldCommand extends CommandBase {
    private final Shoulder shoulder;

    public ShoulderHoldCommand(Shoulder shoulder) {
        this.shoulder = shoulder;

        addRequirements(shoulder);
    }

    @Override
    public void execute() {
        shoulder.runWithSetpoint(ArmState.getTarget().getShoulderAngle(), 0, 0);
    }
}
