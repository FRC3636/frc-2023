// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmHoldCommand;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.MoveNodeSelection;
import frc.robot.commands.MoveNodeSelection.MovementDirection;
import frc.robot.commands.alignment.AlignToClosestNode;
import frc.robot.commands.alignment.AlignToSelectedNode;
import frc.robot.commands.alignment.DriveToNode;
import frc.robot.commands.autonomous.AutoBalance;
import frc.robot.commands.autonomous.AutoScore;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.GameInfoTable;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Rollers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.*;

import javax.xml.crypto.Data;
import java.util.Set;

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
    private final AutoBalance autoBalanceCommand = new AutoBalance(drivetrain);

    // RGB
    public static final GameInfoTable gameInfo = new GameInfoTable();

    // Node
    public Node targetNode = new Node(0);
    public static Field2d nodeSelector = new Field2d();

    // Auto Selection
    private final FieldObject2d startingPosition = field.getObject("Starting Position");
    private final AutoLanguage.AutoProgram autoProgram = new AutoLanguage.AutoProgram(Constants.AutoConstants.DEFAULT_PROGRAM);

    public RobotContainer() {
        autoTab.add("Field", field).withWidget(BuiltInWidgets.kField).withSize(5, 3);
        autoTab.add("Auto Program", autoProgram);
        startingPosition.setPose(0, 0, new Rotation2d());

        armTab.add("Node Selector", nodeSelector).withWidget(BuiltInWidgets.kField).withSize(3, 3);

        driveSettingsTab.addNumber("Turn Sensitivity", RobotContainer.joystickRight::getZ);
        driveSettingsTab.addNumber("Drive Sensitivity", RobotContainer.joystickLeft::getZ);

        LiveWindow.enableTelemetry(arm);
        LiveWindow.enableTelemetry(drivetrain);

        if (!DriverStation.isFMSAttached()) {
            PathPlannerServer.startServer(5811);
        }

        arm.setDefaultCommand(new ArmHoldCommand(arm));

        drivetrain.setDefaultCommand(driveCommand);

        LiveWindow.enableTelemetry(arm);

        configureButtonBindings();

        DriverStation.silenceJoystickConnectionWarning(Robot.isSimulation());

        startingPosition.setPose(AllianceUtils.allianceToField(new Pose2d(new Translation2d(3.47, 0.73), Rotation2d.fromRadians(Math.PI))));

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
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
        new JoystickButton(joystickRight, 4)
                .whileTrue(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new AlignToSelectedNode(drivetrain, arm, poseEstimation, () -> this.targetNode),
                                        new RunCommand(drivetrain::setX, drivetrain)
                                )
                        )
                );

        new JoystickButton(joystickLeft, 4)
                .whileTrue(
                        new SequentialCommandGroup(
                                new GenerateCommand(
                                        () -> new AutoScore(drivetrain, arm, poseEstimation, () -> this.targetNode),
                                        Set.of(drivetrain)
                                        ),
                                new RunCommand(drivetrain::setX, drivetrain)
                        )
                );

        new JoystickButton(joystickRight, 3).whileTrue(
                new SequentialCommandGroup(
                        new GenerateCommand(
                                () -> new DriveToNode(drivetrain, poseEstimation, targetNode, Double.POSITIVE_INFINITY),
                                Set.of(drivetrain)
                        ),
                        new RunCommand(drivetrain::setX, drivetrain)
                )
        );

        new JoystickButton(joystickLeft, 2)
                .whileTrue(autoBalanceCommand);

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
        new Trigger(() -> controller.getLeftTriggerAxis() > 0.05)
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
        DigitalInput armButton = new DigitalInput(1);
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
        autoProgram.recompile();
        return autoProgram
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
