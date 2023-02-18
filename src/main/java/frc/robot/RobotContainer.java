// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCommand;
import frc.robot.vision.PoseEstimation;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    // Dashboard
    public static final ShuffleboardTab driveSettingsTab = Shuffleboard.getTab("Drive Settings");
    public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    public static final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");

    // Subsystems
    public static final Drivetrain drivetrain = new Drivetrain();

    // Pose Estimation
    public static final PoseEstimation poseEstimation = new PoseEstimation();

    // Controllers
    public static final Joystick joystickLeft = new Joystick(Constants.ControlConstants.JOYSTICK_RIGHT_PORT);
    public static final Joystick joystickRight = new Joystick(Constants.ControlConstants.JOYSTICK_LEFT_PORT);
  
    public static final SendableChooser<String> drivePresetsChooser = new SendableChooser<>();
    private static NetworkTableEntry driveSchemeEntry;

    public static Field2d field = new Field2d();


    public RobotContainer() {
        configureButtonBindings();

        autoTab.add("Field", field).withWidget(BuiltInWidgets.kField).withSize(5, 3);
        driveSettingsTab.add("Reset Gyro", new InstantCommand(drivetrain::zeroHeading));

        driveSettingsTab.addNumber("Turn Sensitivity", RobotContainer.joystickRight::getZ);
        driveSettingsTab.addNumber("Drive Sensitivity", RobotContainer.joystickLeft::getZ);

        // FIXME: don't run on FMS
        PathPlannerServer.startServer(5811);

        drivetrain.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> drivetrain.drive(
                                //add 1 to prevent negative sensitivity
                                MathUtil.applyDeadband(-joystickLeft.getY() * (joystickLeft.getZ()+ 1)/2, 0.15),
                                MathUtil.applyDeadband(-joystickLeft.getX() * (joystickLeft.getZ()+1)/2, 0.15),
                                MathUtil.applyDeadband(-joystickRight.getX() * (joystickRight.getZ()+1)/2, 0.15),
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
        new JoystickButton(joystickLeft, 6)
                .onTrue(new InstantCommand(drivetrain::zeroHeading));
    }


    public Command getAutonomousCommand() {
        return AutoCommand.makeAutoCommand(drivetrain, poseEstimation, "Basic");
    }
}
