package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;

public class ShoulderHoldCommand extends CommandBase {
    private final Shoulder shoulder;

    private double position;

    public ShoulderHoldCommand(Shoulder shoulder) {
        this.shoulder = shoulder;

        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        position = shoulder.getActualPosition();
    }

    @Override
    public void execute() {
        shoulder.runWithSetpoint(shoulder.targetPosition, 0, 0);
    }
}
