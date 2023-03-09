package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.Constants;

public class ArmMoveCommand extends CommandBase {
    private final Arm arm;

    private final Timer timer = new Timer();
    private TrapezoidProfile profile;

    public ArmMoveCommand(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Shoulder Goal Position", Arm.State.getTarget().getShoulderAngle().getRadians());

        profile = new TrapezoidProfile(
            Constants.Shoulder.TRAPEZOID_PROFILE_CONSTRAINTS,
            new TrapezoidProfile.State(Arm.State.getTarget().getShoulderAngle().getRadians(), 0),
            new TrapezoidProfile.State(arm.getShoulderAngle().getRadians(), arm.getShoulderVelocity().getRadians())
        );

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        TrapezoidProfile.State state = profile.calculate(timer.get());
        SmartDashboard.putNumber("Shoulder State Goal Position", state.position);
        arm.runWithSetpoint(Rotation2d.fromRadians(state.position), Rotation2d.fromRadians(state.velocity));
    }

    @Override
    public boolean isFinished() {
        return profile.isFinished(timer.get());
    }
}
