package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.ArmState;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class IntakeCommand extends ParallelCommandGroup {
    public IntakeCommand(Rollers rollers, Shoulder shoulder, Wrist wrist, ArmState armState) {
        super(
                new InstantCommand(() -> {ArmState.target = armState;}),
                new FunctionalCommand(
                        rollers::intake,
                        () -> {},
                        interrupted -> {rollers.stop();},
                        () -> false,
                        rollers),
                new ArmMoveCommand(shoulder, wrist).asProxy() // Allows the command to end before the rest of the command group
        );
    }
}
