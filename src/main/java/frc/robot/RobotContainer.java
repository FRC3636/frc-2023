// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.commands.MoveToPoint;
import frc.robot.vision.PoseEstimation;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.drivetrain.Drivetrain;

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

    public static Field2d field = new Field2d();

    // Commands
    private DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, poseEstimation, joystickLeft, joystickRight);

    public RobotContainer() {
        configureButtonBindings();

        autoTab.add("Field", field).withWidget(BuiltInWidgets.kField).withSize(5, 3);

        driveSettingsTab.addNumber("Turn Sensitivity", RobotContainer.joystickRight::getZ);
        driveSettingsTab.addNumber("Drive Sensitivity", RobotContainer.joystickLeft::getZ);

        // FIXME: don't run on FMS
        PathPlannerServer.startServer(5811);


        drivetrain.setDefaultCommand(driveCommand);
    }

    private void configureButtonBindings() {
        new JoystickButton(joystickRight, 1)
                .whileTrue(new RunCommand(
                        drivetrain::setX,
                        drivetrain));
        new JoystickButton(joystickLeft, 6)
                .onTrue(new InstantCommand(driveCommand::resetFieldOrientation));

        Pose2d aprilTagTarget = Constants.FieldConstants.aprilTags.get(Integer.valueOf(3)).toPose2d();
        new JoystickButton(joystickLeft, 1)
                .whileTrue(new MoveToPoint(
                        drivetrain,
                        poseEstimation,
                                aprilTagTarget
                                .transformBy(new Transform2d(
                                        new Translation2d(-1.0, aprilTagTarget.getRotation()),
                                        new Rotation2d()
                                ))));
    }


    public Command getAutonomousCommand() {
        return AutoCommand.makeAutoCommand(drivetrain, poseEstimation, "Basic");
    }
}
