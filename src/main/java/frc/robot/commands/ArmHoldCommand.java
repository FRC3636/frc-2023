package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ArmHoldCommand extends CommandBase {
    private final Arm arm;

    public ArmHoldCommand(Arm arm) {
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.runWithSetpoint(arm.getTargetShoulderAngle(), new Rotation2d());
    }
}
