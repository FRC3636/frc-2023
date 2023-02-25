// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmHoldCommand;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.*;

public class RobotContainer {
    // Dashboard
    public static final ShuffleboardTab driveSettingsTab = Shuffleboard.getTab("Drive Settings");
    public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    public static final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
    public static final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

    // Subsystems
    public static final Drivetrain drivetrain = new Drivetrain();
    public static final Arm arm = new Arm();
    public static final Rollers rollers = new Rollers();

    // Controllers
    public static final Joystick joystickLeft = new Joystick(Constants.ControlConstants.JOYSTICK_RIGHT_PORT);
    public static final Joystick joystickRight = new Joystick(Constants.ControlConstants.JOYSTICK_LEFT_PORT);
    public static final XboxController controller = new XboxController(Constants.ControlConstants.CONTROLLER_PORT);

    public static final SendableChooser<String> drivePresetsChooser = new SendableChooser<>();
    private static GenericEntry driveSchemeEntry;

    public static Field2d field = new Field2d();

    // RGB
    public static final LightsTable lights = new LightsTable();

    static {
        drivePresetsChooser.addOption("Default", DriveConfig.DEFAULT_PRESET_NAME);
        drivePresetsChooser.addOption("Jude", "jude");
        drivePresetsChooser.addOption("Tank Drive", "tank_drive");
        drivePresetsChooser.addOption("Arcade Single", "arcade_single");
    }

    public RobotContainer() {
        driveSettingsTab.add("Drive Presents", drivePresetsChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser);
        driveSchemeEntry = driveSettingsTab.add("Drive Scheme", "None").withWidget(BuiltInWidgets.kTextView).getEntry();
        autoTab.add("Field", field).withWidget(BuiltInWidgets.kField).withSize(5, 3);
        LiveWindow.enableTelemetry(arm);

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

            arm.setDefaultCommand(new ArmHoldCommand(arm));
            // Intaking and Outtaking
      new JoystickButton(controller, XboxController.Button.kRightBumper.value)
              .onTrue(new InstantCommand(rollers::intake))
              .onFalse(new InstantCommand(rollers::stop));

      new JoystickButton(joystickRight, 1)
              .onTrue(new InstantCommand(rollers::outtake))
              .onFalse(new InstantCommand(rollers::stop));

      // State Changes
      new JoystickButton(controller, XboxController.Button.kLeftBumper.value)
              .whileTrue(new InstantCommand(() -> Arm.State.setGamePiece(Arm.State.GamePiece.Cone)));
      new Trigger(() -> controller.getLeftTriggerAxis() > 0.05)
              .whileTrue(new InstantCommand(() -> Arm.State.setGamePiece(Arm.State.GamePiece.Cube)));

      new JoystickButton(controller, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> {Arm.State.setTarget(Arm.State.Stowed);}));
      new JoystickButton(controller, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> {Arm.State.setTarget(Arm.State.Low);}));
      new JoystickButton(controller, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> {Arm.State.setTarget(Arm.State.Mid);}));
      new JoystickButton(controller, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> {Arm.State.setTarget(Arm.State.High);}));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
