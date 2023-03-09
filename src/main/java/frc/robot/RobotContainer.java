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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.MoveNodeSelection.MovementDirection;
import frc.robot.commands.alignment.AlignToSelectedNode;
import frc.robot.commands.autonomous.AutoBalance;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.LightsTable;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Rollers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.Node;

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
    public static final Joystick buttonPanel = new Joystick(Constants.ControlConstants.BUTTON_PANEL_PORT);

    // Pose Estimation
    public static final PoseEstimation poseEstimation = new PoseEstimation();

    public static final SendableChooser<String> drivePresetsChooser = new SendableChooser<>();

    public static Field2d field = new Field2d();

    private GenericEntry autoNodeSelector = autoTab.add("Auto Node", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    private GenericEntry performAutoBalance = autoTab.add("Perform Auto Balance", false)
            .withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    private final FieldObject2d startingPosition = field.getObject("Starting Position");
    private final FieldObject2d autoBalanceStartingPosition = field.getObject("Auto Balance Starting Position");

    // Commands
    private DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, poseEstimation, joystickLeft,
            joystickRight);
    private AutoBalance autoBalanceCommand = new AutoBalance(drivetrain);

    // RGB
    public static final LightsTable lights = new LightsTable();

    // Movement Command
    public Node targetNode = new Node(0);

    public RobotContainer() {
        configureButtonBindings();

        autoTab.add("Field", field).withWidget(BuiltInWidgets.kField).withSize(5, 3);

        driveSettingsTab.addNumber("Turn Sensitivity", RobotContainer.joystickRight::getZ);
        driveSettingsTab.addNumber("Drive Sensitivity", RobotContainer.joystickLeft::getZ);

        LiveWindow.enableTelemetry(arm);

        if (!DriverStation.isFMSAttached()) {
            PathPlannerServer.startServer(5811);
        }

        arm.setDefaultCommand(new ArmHoldCommand(arm));

        for (Arm.State value : Arm.State.values()) {
            armTab.addDoubleArray(value.name(), () -> Arm.State.getPresets(value));
        }

        drivetrain.setDefaultCommand(driveCommand);

        if (autoBalanceStartingPosition.getPoses().isEmpty()) {
            autoBalanceStartingPosition.setPose(
                    AllianceUtils.allianceToField(
                            new Pose2d(
                                    new Translation2d(
                                            0,
                                            0),
                                    new Rotation2d())));
        }
    }

    private void configureButtonBindings() {
        // Pose Estimation
        new JoystickButton(joystickLeft, 6)
                .onTrue(new InstantCommand(driveCommand::resetFieldOrientation));
        new JoystickButton(joystickLeft, 7)
                .onTrue(new InstantCommand(() -> poseEstimation.resetPose(
                        new Pose2d(
                                poseEstimation.getEstimatedPose().getTranslation(),
                                new Rotation2d()))));

        // Driving
        new JoystickButton(joystickLeft, 1)
                .whileTrue(new RunCommand(
                        drivetrain::setX,
                        drivetrain));

        new JoystickButton(joystickRight, 4)
                .whileTrue(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new AlignToSelectedNode(drivetrain, poseEstimation, () -> this.targetNode)
                                // new RunCommand(drivetrain::setX, drivetrain)
                                ),
                                new InstantCommand(() -> {
                                    Arm.State.setTargetFromNode(this.targetNode);
                                })));

        new JoystickButton(joystickRight, 3).whileTrue(
                new SequentialCommandGroup(
                        new AlignToSelectedNode(drivetrain, poseEstimation, () -> this.targetNode),
                        new RunCommand(drivetrain::setX, drivetrain)));

        new JoystickButton(joystickLeft, 2)
                .whileTrue(autoBalanceCommand);

        // Rollers
        new JoystickButton(controller, XboxController.Button.kRightBumper.value)
                .onTrue(new InstantCommand(() -> {
                    Arm.State.setRollerState(Rollers.State.Intake);
                }))
                .onFalse(new InstantCommand(() -> {
                    Arm.State.setRollerState(Rollers.State.Off);
                }));

        new JoystickButton(joystickRight, 1)
                .onTrue(new InstantCommand(() -> {
                    Arm.State.setRollerState(Rollers.State.Outtake);
                }))
                .onFalse(new InstantCommand(() -> {
                    Arm.State.setRollerState(Rollers.State.Off);
                }));

        // Arm Control
        new JoystickButton(controller, XboxController.Button.kLeftBumper.value)
                .whileTrue(new InstantCommand(() -> Arm.State.setGamePiece(Arm.State.GamePiece.Cube)));
        new Trigger(() -> controller.getLeftTriggerAxis() > 0.05)
                .whileTrue(new InstantCommand(() -> Arm.State.setGamePiece(Arm.State.GamePiece.Cone)));

        new JoystickButton(controller, XboxController.Button.kRightStick.value)
                .onTrue(new InstantCommand(Arm.State::resetOffset));
        new Trigger(() -> controller.getPOV() == 0)
                .onTrue(new InstantCommand(() -> Arm.State.moveShoulderOffset(Rotation2d.fromDegrees(2))));
        new Trigger(() -> controller.getPOV() == 180)
                .onTrue(new InstantCommand(() -> Arm.State.moveShoulderOffset(Rotation2d.fromDegrees(-2))));
        new Trigger(() -> controller.getPOV() == 90)
                .onTrue(new InstantCommand(() -> Arm.State.moveWristOffset(Rotation2d.fromDegrees(2))));
        new Trigger(() -> controller.getPOV() == 270)
                .onTrue(new InstantCommand(() -> Arm.State.moveWristOffset(Rotation2d.fromDegrees(-2))));

        new JoystickButton(controller, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> {
            Arm.State.setTarget(Arm.State.Stowed);
        }));
        new JoystickButton(controller, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> {
            Arm.State.setTarget(Arm.State.Low);
        }));
        new JoystickButton(controller, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> {
            Arm.State.setTarget(Arm.State.Mid);
        }));
        new JoystickButton(controller, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> {
            Arm.State.setTarget(Arm.State.High);
        }));
        new JoystickButton(controller, XboxController.Button.kStart.value).onTrue(new InstantCommand(() -> {
            Arm.State.setTarget(Arm.State.Slide);
        }));
        new JoystickButton(controller, XboxController.Button.kBack.value).onTrue(new InstantCommand(() -> {
            Arm.State.setTarget(Arm.State.Teller);
        }));

        // Node Selector
        for (int i = 0; i < 9; i++) {
            int finalI = i;
            new JoystickButton(buttonPanel, i + 1)
                    .onTrue(new InstantCommand(() -> this.setTargetNode(new Node(finalI))));
        }
        new JoystickButton(joystickRight, 2)
                .onTrue(new InstantCommand(() -> this.setTargetNode(new Node((int) autoNodeSelector.getInteger(0)))));

        new Trigger(() -> controller.getLeftX() >= 0.5)
                .onTrue(new MoveNodeSelection(this, MovementDirection.Left));
        new Trigger(() -> controller.getLeftX() <= -0.5)
                .onTrue(new MoveNodeSelection(this, MovementDirection.Right));
        new Trigger(() -> controller.getLeftY() >= 0.5).onTrue(new MoveNodeSelection(this, MovementDirection.Up));
        new Trigger(() -> controller.getLeftY() <= -0.5).onTrue(new MoveNodeSelection(this, MovementDirection.Down));
        new JoystickButton(joystickRight, 2)
                .onTrue(new InstantCommand(() -> this.setTargetNode(new Node((int) autoNodeSelector.getInteger(0)))));

        // calibration movement routine
        new JoystickButton(joystickLeft, 5).onTrue(new NavigateToPoint(drivetrain, poseEstimation, poseEstimation
                .getEstimatedPose().transformBy(new Transform2d(new Translation2d(0, 4), Rotation2d.fromRadians(0)))));
        new JoystickButton(joystickRight, 5).onTrue(new NavigateToPoint(drivetrain, poseEstimation, poseEstimation
                .getEstimatedPose().transformBy(new Transform2d(new Translation2d(0, -4), Rotation2d.fromRadians(0)))));
    }

    public Command getAutonomousCommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        Pose2d startingPose = startingPosition.getPose();

        command.addCommands(new InstantCommand(() -> poseEstimation.resetPose(startingPose)));

        return AutoCommand.makeAutoCommand(drivetrain, poseEstimation, "test");
    }

    public void setTargetNode(Node targetNode) {
        RobotContainer.field.getObject("Node Position").setPose(targetNode.getNodePose());
        this.targetNode = targetNode;
    }
}
