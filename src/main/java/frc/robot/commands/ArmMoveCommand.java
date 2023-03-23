package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

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
        SmartDashboard.putNumber("Shoulder Goal Position", arm.getTargetShoulderAngle().getRadians());

        profile = generateProfile(arm.getTarget(), arm);
        SmartDashboard.putNumber("Trajectory Time", profile.totalTime());

        timer.reset();
        timer.start();
    }

    public static TrapezoidProfile generateProfile(Arm.State goal, Arm arm){
        TrapezoidProfile profile = new TrapezoidProfile(Constants.Shoulder.TRAPEZOID_PROFILE_CONSTRAINTS, 
        new TrapezoidProfile.State(goal.getShoulderAngleFor(arm.getGamePiece()).getRadians(), 0),
        new TrapezoidProfile.State(arm.getShoulderAngle().getRadians(), arm.getShoulderVelocity().getRadians()));
        return profile;
    }

    @Override
    public void execute() {
        TrapezoidProfile.State state = profile.calculate(timer.get());
        SmartDashboard.putNumber("Shoulder State Goal Position", state.position);
        arm.runWithSetpoint(Rotation2d.fromRadians(state.position), Rotation2d.fromRadians(state.velocity));
    }

    public static boolean pathIntersectsChargeStation(Arm.State goal, Arm arm) {
        return (
                goal.getShoulderAngleFor(arm.getGamePiece()).getRadians() >= Constants.Shoulder.MID_CUBE_ANGLE.getRadians() &&
                arm.getShoulderAngle().getRadians() <= Constants.Shoulder.MID_CUBE_ANGLE.getRadians()
        ) || (
                goal.getShoulderAngleFor(arm.getGamePiece()).getRadians() <= Constants.Shoulder.MID_CUBE_ANGLE.getRadians() &&
                arm.getShoulderAngle().getRadians() >= Constants.Shoulder.MID_CUBE_ANGLE.getRadians()
        );
    }

    @Override
    public boolean isFinished() {
        return profile.isFinished(timer.get());
    }
}
