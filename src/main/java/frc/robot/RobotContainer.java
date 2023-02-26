// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.commands.AutoCommand;
import frc.robot.commands.MoveToPoint;
import frc.robot.commands.ArmHoldCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Rollers;
import frc.robot.vision.PoseEstimation;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.*;

public class RobotContainer {
    // Dashboard
    public static final ShuffleboardTab driveSettingsTab = Shuffleboard.getTab("Drive Settings");
    public static final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    public static final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
    public static final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

    // Subsystems
    public static final Drivetrain drivetrain = new Drivetrain();
    public static final Arm arm = new Arm();

    // Controllers
    public static final Joystick joystickLeft = new Joystick(Constants.ControlConstants.JOYSTICK_RIGHT_PORT);
    public static final Joystick joystickRight = new Joystick(Constants.ControlConstants.JOYSTICK_LEFT_PORT);
    public static final XboxController controller = new XboxController(Constants.ControlConstants.CONTROLLER_PORT);

    // Pose Estimation
    public static final PoseEstimation poseEstimation = new PoseEstimation();

    public static final SendableChooser<String> drivePresetsChooser = new SendableChooser<>();

    public static Field2d field = new Field2d();

    // Commands
    private DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, poseEstimation, joystickLeft, joystickRight);

    // RGB
    public static final LightsTable lights = new LightsTable();

    public RobotContainer() {
        configureButtonBindings();

        autoTab.add("Field", field).withWidget(BuiltInWidgets.kField).withSize(5, 3);

        driveSettingsTab.addNumber("Turn Sensitivity", RobotContainer.joystickRight::getZ);
        driveSettingsTab.addNumber("Drive Sensitivity", RobotContainer.joystickLeft::getZ);

        LiveWindow.enableTelemetry(arm);

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
      arm.setDefaultCommand(new ArmHoldCommand(arm));

      // Intaking and Outtaking
      new JoystickButton(controller, XboxController.Button.kRightBumper.value)
              .onTrue(new InstantCommand(() -> {Arm.State.setRollerState(Rollers.State.Intake);}))
              .onFalse(new InstantCommand(() -> {Arm.State.setRollerState(Rollers.State.Off);}));

      new JoystickButton(joystickRight, 1)
              .onTrue(new InstantCommand(() -> {Arm.State.setRollerState(Rollers.State.Outtake);}))
              .onFalse(new InstantCommand(() -> {Arm.State.setRollerState(Rollers.State.Off);}));

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
        return AutoCommand.makeAutoCommand(drivetrain, poseEstimation, "Basic");
    }
}
