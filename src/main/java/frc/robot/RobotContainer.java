// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

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


  public RobotContainer() {
    configureButtonBindings();

    drivetrain.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drivetrain.drive(
                MathUtil.applyDeadband(-joystickLeft.getY(), 0.06),
                MathUtil.applyDeadband(-joystickLeft.getX(), 0.06),
                MathUtil.applyDeadband(-joystickRight.getX(), 0.06),
                true),
                drivetrain
        )
    );
  }

  private void configureButtonBindings() {
    new JoystickButton(joystickRight, 1)
        .whileTrue(new RunCommand(
                drivetrain::setX,
                drivetrain));
  }


  public Command getAutonomousCommand() {
    return null;
  }
}
