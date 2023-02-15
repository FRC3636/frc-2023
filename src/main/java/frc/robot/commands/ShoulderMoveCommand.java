package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shoulder;

public class ShoulderMoveCommand extends CommandBase {
    private final Shoulder shoulder;
    private final double goalPosition;

    private final Timer timer = new Timer();
    private TrapezoidProfile profile;

    public ShoulderMoveCommand(Shoulder shoulder, double goalPosition) {
        this.shoulder = shoulder;
        this.goalPosition = goalPosition;

        SmartDashboard.putNumber("Shoulder Goal Position", goalPosition);

        addRequirements(shoulder);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Shoulder Goal Position", goalPosition);

        profile = new TrapezoidProfile(
            Constants.Shoulder.TRAPEZOID_PROFILE_CONSTRAINTS,
            new TrapezoidProfile.State(goalPosition, 0),
            new TrapezoidProfile.State(shoulder.getActualPosition(), shoulder.getActualVelocity())
        );

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        TrapezoidProfile.State state = profile.calculate(timer.get());
        shoulder.runWithSetpoint(state.position, state.velocity, 0);
    }

    @Override
    public boolean isFinished() {
        return profile.isFinished(timer.get());
    }
}
