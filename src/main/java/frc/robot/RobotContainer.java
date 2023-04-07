// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.*;
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
import frc.robot.commands.ArmHoldCommand;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.MoveNodeSelection;
import frc.robot.commands.MoveNodeSelection.MovementDirection;
import frc.robot.commands.alignment.AlignToClosestNode;
import frc.robot.commands.alignment.AlignToSelectedNode;
import frc.robot.commands.Balance;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.GameInfoTable;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Rollers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.AutoLanguage;
import frc.robot.utils.GamePiece;
import frc.robot.utils.Node;

import java.util.Optional;

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
    public static Field2d field = new Field2d();

    // Commands
    private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, poseEstimation, joystickLeft,
            joystickRight);
    private final Balance autoBalanceCommand = new Balance(drivetrain);

    // RGB
    public static final GameInfoTable gameInfo = new GameInfoTable();

    // Node
    public Node targetNode = new Node(0);
    public static Field2d nodeSelector = new Field2d();

    // Auto Selection
    private final FieldObject2d startingPosition = field.getObject("Starting Position");
    private final SendableChooser<AutoLanguage.AutoProgram> autoSelector = new SendableChooser<>() {
        @Override
        public void initSendable(SendableBuilder builder) {
            super.initSendable(builder);
            builder.addStringProperty("Auto Program", () -> this.getSelected().getProgram(), (program) -> autoSelector.getSelected().setProgram(program));
            builder.addStringProperty("Auto Compilation Error", () -> this.getSelected().getCompilationError().map(Exception::getMessage).orElse(""), (e) -> {});
            builder.addBooleanProperty("Auto Compilation Good", () -> this.getSelected().getCompilationOutput().isPresent(), (g) -> {});
        }
    };

    public RobotContainer() {
        if (!DriverStation.isFMSAttached()) {
            PathPlannerServer.startServer(5811);
        }
        DriverStation.silenceJoystickConnectionWarning(Robot.isSimulation());

        setDefaultCommands();
        configureAutoTab();
        configureButtonBindings();

        enableLogging();
    }

    private void enableLogging() {
        LiveWindow.enableTelemetry(arm);
        LiveWindow.enableTelemetry(drivetrain);

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    private void setDefaultCommands() {
        arm.setDefaultCommand(new ArmHoldCommand(arm));
        drivetrain.setDefaultCommand(driveCommand);
    }

    private void configureAutoTab() {
        autoTab.add("Field", field).withWidget(BuiltInWidgets.kField).withSize(5, 3);
        autoSelector.setDefaultOption("Cube Low", new AutoLanguage.AutoProgram(Constants.AutoConstants.DEFAULT_PROGRAM));
        autoSelector.addOption("Grid 1: 2.5 Piece", new AutoLanguage.AutoProgram("score cone 1 high cone_right; intake cube 1; score cube 1 high cube; intake cone 2;"));
        autoSelector.addOption("Grid 2: 1.5 + Balance", new AutoLanguage.AutoProgram("score cone 2 high cone_right; intake cube 2 ignore_obstacles; balance;"));
        autoSelector.addOption("Grid 3: 2.5 Piece", new AutoLanguage.AutoProgram("score cone 3 high cone_right; intake cube 4; score cube 1 high cube; intake cone 3;"));

        startingPosition.setPose(AllianceUtils.allianceToField(new Pose2d(new Translation2d(3.47, 0.73), Rotation2d.fromRadians(Math.PI))));
        autoTab.add("Set Starting Position", new InstantCommand(() -> startingPosition.setPose(poseEstimation.getEstimatedPose())) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        }).withWidget(BuiltInWidgets.kCommand);

        autoTab.add("Auto Chooser", autoSelector);

        armTab.add("Node Selector", nodeSelector).withWidget(BuiltInWidgets.kField).withSize(3, 3);
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

        // Auto Alignment
        new JoystickButton(joystickLeft, 4)
                .onTrue(new InstantCommand(() -> targetNode = targetNode.getWithGridOverride(Node.Grid.left)))
                .onFalse(new InstantCommand(() -> targetNode = targetNode.getWithGridOverride(Optional.empty())));

        new JoystickButton(joystickLeft, 3)
                .onTrue(new InstantCommand(() -> targetNode = targetNode.getWithGridOverride(Node.Grid.center)))
                .onFalse(new InstantCommand(() -> targetNode = targetNode.getWithGridOverride(Optional.empty())));


        new JoystickButton(joystickLeft, 5)
                .onTrue(new InstantCommand(() -> targetNode = targetNode.getWithGridOverride(Node.Grid.right)))
                .onFalse(new InstantCommand(() -> targetNode = targetNode.getWithGridOverride(Optional.empty())));

        new JoystickButton(joystickRight, 4)
                .whileTrue(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new AlignToSelectedNode(drivetrain, arm, poseEstimation, () -> this.targetNode.getWithColumn(Node.Column.LeftCone)),
                                        new RunCommand(drivetrain::setX, drivetrain)
                                )
                        )
                );

        new JoystickButton(joystickRight, 3)
                .whileTrue(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new AlignToSelectedNode(drivetrain, arm, poseEstimation, () -> this.targetNode.getWithColumn(Node.Column.Cube)),
                                        new RunCommand(drivetrain::setX, drivetrain)
                                )
                        )
                );

        new JoystickButton(joystickRight, 5)
                .whileTrue(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new AlignToSelectedNode(drivetrain, arm, poseEstimation, () -> this.targetNode.getWithColumn(Node.Column.RightCone)),
                                        new RunCommand(drivetrain::setX, drivetrain)
                                )
                        )
                );


        // Auto
        new JoystickButton(joystickLeft, 2)
                .whileTrue(autoBalanceCommand);

        new JoystickButton(controller, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> {
            targetNode = targetNode.getWithLevel(Node.Level.Low);
        }));
        new JoystickButton(controller, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> {
            targetNode = targetNode.getWithLevel(Node.Level.Mid);
        }));
        new JoystickButton(controller, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> {
            targetNode = targetNode.getWithLevel(Node.Level.High);
        }));

        // Rollers
        new JoystickButton(controller, XboxController.Button.kRightBumper.value)
                .onTrue(new InstantCommand(() -> {
                    if (!joystickRight.getRawButton(1)) {
                        arm.setRollerState(Rollers.State.Intake);
                    }
                }))
                .onFalse(new InstantCommand(() -> {
                    if (!joystickRight.getRawButton(1)) {
                        arm.setRollerState(Rollers.State.Off);
                    }
                }));

        new JoystickButton(joystickRight, 1)
                .onTrue(new InstantCommand(() -> {
                    arm.setRollerState(Rollers.State.Outtake);
                }))
                .onFalse(new InstantCommand(() -> {
                    if (controller.getRawButton(XboxController.Button.kRightBumper.value)) {
                        arm.setRollerState(Rollers.State.Intake);
                    } else {
                        arm.setRollerState(Rollers.State.Off);
                    }
                }));

        new JoystickButton(joystickRight, 2).whileTrue(new AlignToClosestNode(drivetrain, arm, poseEstimation));

        // Arm Control
        new JoystickButton(controller, XboxController.Button.kLeftBumper.value)
                .whileTrue(new InstantCommand(() -> arm.setGamePiece(GamePiece.Cube)));
        new Trigger(() -> controller.getLeftTriggerAxis() > 0.2)
                .whileTrue(new InstantCommand(() -> arm.setGamePiece(GamePiece.Cone)));

        new JoystickButton(controller, XboxController.Button.kRightStick.value)
                .onTrue(new InstantCommand(arm::resetOffset));

        new Trigger(() -> controller.getPOV() == 0)
                .onTrue(new InstantCommand(() -> arm.moveHeightOffset(0.01)));
        new Trigger(() -> controller.getPOV() == 180)
                .onTrue(new InstantCommand(() -> arm.moveHeightOffset(-0.01)));
        new Trigger(() -> controller.getPOV() == 90)
                .onTrue(new InstantCommand(() -> arm.moveShoulderOffset(Rotation2d.fromDegrees(2))));
        new Trigger(() -> controller.getPOV() == 270)
                .onTrue(new InstantCommand(() -> arm.moveShoulderOffset(Rotation2d.fromDegrees(-2))));

        new Trigger(() -> controller.getRightTriggerAxis() > 0.05)
                .whileTrue(
                        new RunCommand(() -> arm.setTemporaryAngleOffset(Rotation2d.fromRadians(controller.getRightTriggerAxis() / 1.5))))
                .onFalse(new InstantCommand(() -> arm.setTemporaryAngleOffset(new Rotation2d())));

        new JoystickButton(controller, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> {
            arm.setTarget(Arm.State.Stowed);
        }));
        new JoystickButton(controller, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> {
            arm.setTarget(Arm.State.Low);
        }));
        new JoystickButton(controller, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> {
            arm.setTarget(Arm.State.Mid);
        }));
        new JoystickButton(controller, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> {
            arm.setTarget(Arm.State.High);
        }));
        new JoystickButton(controller, XboxController.Button.kStart.value).onTrue(new InstantCommand(() -> {
            arm.setTarget(Arm.State.Slide);
        }));
        new JoystickButton(controller, XboxController.Button.kBack.value).onTrue(new InstantCommand(() -> {
            arm.setTarget(Arm.State.Teller);
        }));

        // Arm Disabling
        DigitalInput armButton = new DigitalInput(Constants.Arm.UNLOCK_BUTTON_PORT);
        new Trigger(() -> !armButton.get() && DriverStation.isDisabled()).toggleOnTrue(
                new FunctionalCommand(
                        () -> arm.setIdleMode(CANSparkMax.IdleMode.kCoast),
                        () -> {},
                        (interrupted) -> {arm.setIdleMode(CANSparkMax.IdleMode.kBrake);},
                        DriverStation::isEnabled
                        ) {
                    @Override
                    public boolean runsWhenDisabled() {
                        return true;
                    }
                }
        );

        // Node Selector
        new Trigger(() -> controller.getLeftX() >= 0.75)
                .onTrue(new MoveNodeSelection(this, MovementDirection.Left));
        new Trigger(() -> controller.getLeftX() <= -0.75)
                .onTrue(new MoveNodeSelection(this, MovementDirection.Right));
        new Trigger(() -> controller.getLeftY() >= 0.75)
                .onTrue(new MoveNodeSelection(this, MovementDirection.Up));
        new Trigger(() -> controller.getLeftY() <= -0.75)
                .onTrue(new MoveNodeSelection(this, MovementDirection.Down));
    }

    public Command getAutonomousCommand() {
        autoSelector.getSelected().recompile();
        return autoSelector.getSelected()
                .getCompilationOutput()
                .orElse(AutoLanguage.compile(Constants.AutoConstants.DEFAULT_PROGRAM))
                .beforeStarting(() -> poseEstimation.resetPose(startingPosition.getPose()));
    }

    public void setTargetNode(Node targetNode) {
        RobotContainer.field.getObject("Node Position").setPose(targetNode.getNodePose());
        RobotContainer.nodeSelector.setRobotPose(
                targetNode.getColumn().ordinal() * 1d + 0.5d,
                3d - targetNode.getLevel().ordinal() * 1d - 0.5d,
                new Rotation2d()
        );
        this.targetNode = targetNode;
    }
}
