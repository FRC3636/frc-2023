package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.DriveConfig;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import java.util.Set;

public class DriveCommand implements Command {
    private final Drivetrain drivetrain;

    public DriveCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void initialize() {
        // // this method is called once when the command is scheduled
        // // we can use it to reset the DriveTrain's encoders
        // drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        DriveConfig config = DriveConfig.getCurrent();

        final double forward = -RobotContainer.joystickLeft.getY();
        final double turn = RobotContainer.joystickRight.getX();
        final double speedSensitivity = config.getSpeedSensitivity();
        final double turnSensitivity = config.getTurnSensitivity();

        switch (config.getDriveScheme()) {
            case Arcade:
                drivetrain.arcadeDrive(forward / speedSensitivity, turn / turnSensitivity);
                break;
            case ArcadeSingle:
                drivetrain.arcadeDrive(forward / speedSensitivity, RobotContainer.joystickLeft.getX() / turnSensitivity);
                break;
            case Tank:
                drivetrain.tankDrive(forward / speedSensitivity, -RobotContainer.joystickRight.getY() / speedSensitivity);
                break;
        }
    }

    public void end(boolean interrupted) {
        // this method is called when the command ends
        // we can use it to stop the DriveTrain
        drivetrain.tankDrive(0, 0);
    }

    public boolean isFinished() {
        return false;
    }

    public Set<Subsystem> getRequirements() {
        // tell the scheduler that this command requires
        // exclusive control of the DriveTrain
        return Set.of(drivetrain);
    }
}
