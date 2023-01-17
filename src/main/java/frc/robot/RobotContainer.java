// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drivetrain drivetrain = new Drivetrain();
  private final Camera camera = new Camera();

  // Controllers
  public static final Joystick joystickLeft = new Joystick(Constants.Controls.JOYSTICK_RIGHT_PORT);
  public static final Joystick joystickRight = new Joystick(1);

  // Dashboard
  private static final ShuffleboardTab driveSettings = Shuffleboard.getTab("Drive Settings");
  public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

  public static final SendableChooser<String> drivePresetsChooser = new SendableChooser<>();
  private static NetworkTableEntry driveSchemeEntry;

  public static Field2d field = new Field2d();
  public static Field2d testField = new Field2d();


  static {
    drivePresetsChooser.addOption("Default", DriveConfig.DEFAULT_PRESET_NAME);
    drivePresetsChooser.addOption("Person 2", "person_2");
    drivePresetsChooser.addOption("Tank Drive", "tank_drive");
    drivePresetsChooser.addOption("Arcade Single", "arcade_single");
  }

  public RobotContainer() {
    autoTab.add("Field", field).withWidget(BuiltInWidgets.kField).withSize(5, 3);
    autoTab.add("Test Field", testField).withWidget(BuiltInWidgets.kField).withSize(5, 3);

    configureBindings();
  }

  public static void updateDriveSchemeWidget(DriveConfig.DriveScheme driveScheme) {
    if (driveSchemeEntry == null)
      return;
    driveSchemeEntry.setString(driveScheme.toString());
  }


  private void configureBindings() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
