package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;;

public class ZeroWristCommand extends CommandBase {
    private final Wrist wrist;

    public ZeroWristCommand(Wrist wrist) {
        this.wrist = wrist;

        addRequirements(wrist);
    }

    @Override
    public void execute() {
        this.wrist.RunWithVelocity(0.1);
    }
}
