// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShoulderHoldCommand;
import frc.robot.commands.ShoulderMoveCommand;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Dashboard
    private static final ShuffleboardTab driveSettings = Shuffleboard.getTab("Drive Settings");
    public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    public static final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

    // Subsystems
    public static final Drivetrain drivetrain = new Drivetrain();
    // public static final Camera camera = new Camera();
    public static final Shoulder shoulder = new Shoulder();
    public static final Wrist wrist = new Wrist();
    public static final Rollers rollers = new Rollers();

    // Controllers
    public static final Joystick joystickLeft = new Joystick(Constants.Controls.JOYSTICK_LEFT_PORT);
    public static final Joystick joystickRight = new Joystick(Constants.Controls.JOYSTICK_RIGHT_PORT);
    public static final XboxController controller = new XboxController(Constants.Controls.CONTROLLER_PORT);

    public static final SendableChooser<String> drivePresetsChooser = new SendableChooser<>();
    private static GenericEntry driveSchemeEntry;

    public static Field2d field = new Field2d();

    static {
        drivePresetsChooser.addOption("Default", DriveConfig.DEFAULT_PRESET_NAME);
        drivePresetsChooser.addOption("Jude", "jude");
        drivePresetsChooser.addOption("Tank Drive", "tank_drive");
        drivePresetsChooser.addOption("Arcade Single", "arcade_single");
    }

    public RobotContainer() {
        driveSettings.add("Drive Presents", drivePresetsChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser);
        driveSchemeEntry = driveSettings.add("Drive Scheme", "None").withWidget(BuiltInWidgets.kTextView).getEntry();
        autoTab.add("Field", field).withWidget(BuiltInWidgets.kField).withSize(5, 3);

        configureBindings();
    }

    public static void updateDriveSchemeWidget(DriveConfig.DriveScheme driveScheme) {
        if (driveSchemeEntry == null)
            return;
        driveSchemeEntry.setString(driveScheme.toString());
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(new RunCommand(() -> {
            DriveConfig config = DriveConfig.getCurrent();

            final double forward = -RobotContainer.joystickLeft.getY();
            final double turn = -RobotContainer.joystickRight.getX();
            final double speedSensitivity = config.getSpeedSensitivity();
            final double turnSensitivity = config.getTurnSensitivity();

            switch (config.getDriveScheme()) {
                case Arcade:
                    drivetrain.arcadeDrive(forward / speedSensitivity, turn / turnSensitivity);
                    break;
                case ArcadeSingle:
                    drivetrain.arcadeDrive(forward / speedSensitivity,
                            RobotContainer.joystickLeft.getX() / turnSensitivity);
                    break;
                case Tank:
                    drivetrain.tankDrive(forward / speedSensitivity,
                            -RobotContainer.joystickRight.getY() / speedSensitivity);
                    break;
            }
        }, drivetrain));

        // new JoystickButton(joystickLeft, 1).onTrue(new InstantCommand(() -> {
        // if (camera.getRobotPose() != null) {
        // drivetrain.resetOdometryTo(camera.getRobotPose());
        // }
        // }));

        new JoystickButton(controller, XboxController.Button.kRightBumper.value)
                .whileTrue(new IntakeCommand(rollers, shoulder, wrist, Rollers.RollerDirection.IntakeCube, Shoulder.Position.IntakeCube, Wrist.Position.Cube));

        new Trigger(() -> controller.getRightTriggerAxis() > 0.05)
                .onTrue(new IntakeCommand(rollers, shoulder, wrist, Rollers.RollerDirection.IntakeCone, Shoulder.Position.IntakeCone, Wrist.Position.Cone));

       new JoystickButton(controller, XboxController.Button.kStart.value)
               .whileTrue(new RunCommand(wrist::temporaryUpdateWrist));

//        new JoystickButton(controller, XboxController.Button.kBack.value)
//                .whileTrue(new MeasureFF(shoulder));

        shoulder.setDefaultCommand(new ShoulderHoldCommand(shoulder));

        new JoystickButton(controller, XboxController.Button.kX.value).whileTrue(new ShoulderMoveCommand(shoulder, Shoulder.Position.Stowed));
        new JoystickButton(controller, XboxController.Button.kA.value).whileTrue(new ShoulderMoveCommand(shoulder, Shoulder.Position.IntakeCone));
        new JoystickButton(controller, XboxController.Button.kB.value).whileTrue(new ShoulderMoveCommand(shoulder, Shoulder.Position.MidGoalCone));
        new JoystickButton(controller, XboxController.Button.kY.value).whileTrue(new ShoulderMoveCommand(shoulder, Shoulder.Position.HighGoalCone));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
