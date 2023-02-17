package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

import java.util.Set;

public class IntakeCommand extends ParallelCommandGroup {
    public IntakeCommand(Rollers rollers, Shoulder shoulder, Wrist wrist, Rollers.RollerDirection direction, Shoulder.Position armPosition, Wrist.Position wristPosition) {
        super(
                new FunctionalCommand(
                        () -> {rollers.runRollers(direction);},
                        () -> {},
                        interrupted -> {rollers.stop();},
                        () -> false,
                        rollers),
                new ShoulderMoveCommand(shoulder, armPosition).asProxy(), // Allows the command to end before the rest of the command group
                new InstantCommand(() -> {wrist.setTargetPosition(wristPosition);})
        );
    }
}
