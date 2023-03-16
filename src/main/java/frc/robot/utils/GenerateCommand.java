package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;
import java.util.function.Supplier;

public class GenerateCommand implements Command {

    Command inner;
    Supplier<Command> commandSupplier;

    public GenerateCommand(Supplier<Command> commandSupplier) {
        this.commandSupplier = commandSupplier;
    }

    @Override
    public void initialize() {
        inner = commandSupplier.get();
        inner.initialize();
    }

    @Override
    public void execute() {
        inner.execute();
    }

    @Override
    public boolean isFinished() {
        return inner.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        inner.end(interrupted);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return commandSupplier.get().getRequirements();
    }
}
