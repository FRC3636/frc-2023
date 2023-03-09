// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.server.PathPlannerServer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.ArmHoldCommand
import frc.robot.commands.DriveWithJoysticks
import frc.robot.commands.MoveNodeSelection
import frc.robot.commands.MoveNodeSelection.MovementDirection
import frc.robot.commands.NavigateToPoint
import frc.robot.commands.alignment.AlignToSelectedNode
import frc.robot.commands.autonomous.AutoBalance
import frc.robot.poseestimation.PoseEstimation
import frc.robot.subsystems.LightsTable
import frc.robot.subsystems.arm.Arm
import frc.robot.subsystems.arm.Rollers
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.utils.AllianceUtils
import frc.robot.utils.Node

class RobotContainer {
    private val autoNodeSelector = autoTab.add("Auto Node", 0).withWidget(BuiltInWidgets.kTextView).entry
    private val performAutoBalance = autoTab.add("Perform Auto Balance", false)
            .withWidget(BuiltInWidgets.kBooleanBox).entry
    private val startingPosition = field.getObject("Starting Position")
    private val autoBalanceStartingPosition = field.getObject("Auto Balance Starting Position")

    // Commands
    private val driveCommand = DriveWithJoysticks(drivetrain, poseEstimation, joystickLeft,
            joystickRight)
    private val autoBalanceCommand = AutoBalance(drivetrain)

    // Movement Command
    @JvmField
    var targetNode = Node(0)

    init {
        configureButtonBindings()
        autoTab.add("Field", field).withWidget(BuiltInWidgets.kField).withSize(5, 3)
        driveSettingsTab.addNumber("Turn Sensitivity") { joystickRight.z }
        driveSettingsTab.addNumber("Drive Sensitivity") { joystickLeft.z }
        LiveWindow.enableTelemetry(arm)
        if (!DriverStation.isFMSAttached()) {
            PathPlannerServer.startServer(5811)
        }
        arm.defaultCommand = ArmHoldCommand(arm)
        for (value in Arm.State.values()) {
            armTab.addDoubleArray(value.name) { Arm.State.getPresets(value) }
        }
        drivetrain.defaultCommand = driveCommand
        if (autoBalanceStartingPosition.poses.isEmpty()) {
            autoBalanceStartingPosition.pose = AllianceUtils.allianceToField(
                    Pose2d(
                            Translation2d(
                                    0.0,
                                    0.0),
                            Rotation2d()))
        }
    }

    private fun configureButtonBindings() {
        // Pose Estimation
        JoystickButton(joystickLeft, 6)
                .onTrue(InstantCommand({ driveCommand.resetFieldOrientation() }))
        JoystickButton(joystickLeft, 7)
                .onTrue(InstantCommand({
                    poseEstimation.resetPose(
                            Pose2d(
                                    poseEstimation.estimatedPose.translation,
                                    Rotation2d()))
                }))

        // Driving
        JoystickButton(joystickLeft, 1)
                .whileTrue(RunCommand({ drivetrain.setX() },
                        drivetrain))
        JoystickButton(joystickRight, 4)
                .whileTrue(
                        ParallelCommandGroup(
                                SequentialCommandGroup(
                                        AlignToSelectedNode(drivetrain, poseEstimation) { targetNode }
                                ),
                                InstantCommand({ Arm.State.setTargetFromNode(targetNode) })))
        JoystickButton(joystickRight, 3).whileTrue(
                SequentialCommandGroup(
                        AlignToSelectedNode(drivetrain, poseEstimation) { targetNode },
                        RunCommand({ drivetrain.setX() }, drivetrain)))
        JoystickButton(joystickLeft, 2)
                .whileTrue(autoBalanceCommand)

        // Rollers
        JoystickButton(controller, XboxController.Button.kRightBumper.value)
                .onTrue(InstantCommand({ Arm.State.setRollerState(Rollers.State.Intake) }))
                .onFalse(InstantCommand({ Arm.State.setRollerState(Rollers.State.Off) }))
        JoystickButton(joystickRight, 1)
                .onTrue(InstantCommand({ Arm.State.setRollerState(Rollers.State.Outtake) }))
                .onFalse(InstantCommand({ Arm.State.setRollerState(Rollers.State.Off) }))

        // Arm Control
        JoystickButton(controller, XboxController.Button.kLeftBumper.value)
                .whileTrue(InstantCommand({ Arm.State.setGamePiece(Arm.State.GamePiece.Cube) }))
        Trigger { controller.leftTriggerAxis > 0.05 }
                .whileTrue(InstantCommand({ Arm.State.setGamePiece(Arm.State.GamePiece.Cone) }))
        JoystickButton(controller, XboxController.Button.kRightStick.value)
                .onTrue(InstantCommand({ Arm.State.resetOffset() }))
        Trigger { controller.pov == 0 }
                .onTrue(InstantCommand({ Arm.State.moveShoulderOffset(Rotation2d.fromDegrees(2.0)) }))
        Trigger { controller.pov == 180 }
                .onTrue(InstantCommand({ Arm.State.moveShoulderOffset(Rotation2d.fromDegrees(-2.0)) }))
        Trigger { controller.pov == 90 }
                .onTrue(InstantCommand({ Arm.State.moveWristOffset(Rotation2d.fromDegrees(2.0)) }))
        Trigger { controller.pov == 270 }
                .onTrue(InstantCommand({ Arm.State.moveWristOffset(Rotation2d.fromDegrees(-2.0)) }))
        JoystickButton(controller, XboxController.Button.kA.value)
                .onTrue(InstantCommand({ Arm.State.target = Arm.State.Stowed }))
        JoystickButton(controller, XboxController.Button.kB.value)
                .onTrue(InstantCommand({ Arm.State.target = Arm.State.Low }))
        JoystickButton(controller, XboxController.Button.kX.value)
                .onTrue(InstantCommand({ Arm.State.target = Arm.State.Mid }))
        JoystickButton(controller, XboxController.Button.kY.value)
                .onTrue(InstantCommand({ Arm.State.target = Arm.State.High }))
        JoystickButton(controller, XboxController.Button.kStart.value)
                .onTrue(InstantCommand({ Arm.State.target = Arm.State.Slide }))
        JoystickButton(controller, XboxController.Button.kBack.value)
                .onTrue(InstantCommand({ Arm.State.target = Arm.State.Teller }))

        // Node Selector
        for (i in 0..8) {
            JoystickButton(buttonPanel, i + 1)
                    .onTrue(InstantCommand({ setTargetNode(Node(i)) }))
        }

        JoystickButton(joystickRight, 2)
                .onTrue(InstantCommand({ setTargetNode(Node(autoNodeSelector.getInteger(0).toInt())) }))
        Trigger { controller.leftX >= 0.5 }
                .onTrue(MoveNodeSelection(this, MovementDirection.Left))
        Trigger { controller.leftX <= -0.5 }
                .onTrue(MoveNodeSelection(this, MovementDirection.Right))
        Trigger { controller.leftY >= 0.5 }.onTrue(MoveNodeSelection(this, MovementDirection.Up))
        Trigger { controller.leftY <= -0.5 }.onTrue(MoveNodeSelection(this, MovementDirection.Down))
        JoystickButton(joystickRight, 2)
                .onTrue(InstantCommand({ setTargetNode(Node(autoNodeSelector.getInteger(0).toInt())) }))

        // calibration movement routine
        JoystickButton(joystickLeft, 5).onTrue(NavigateToPoint(drivetrain, poseEstimation, poseEstimation
                .estimatedPose.transformBy(Transform2d(Translation2d(0.0, 4.0), Rotation2d.fromRadians(0.0)))))
        JoystickButton(joystickRight, 5).onTrue(NavigateToPoint(drivetrain, poseEstimation, poseEstimation
                .estimatedPose.transformBy(Transform2d(Translation2d(0.0, -4.0), Rotation2d.fromRadians(0.0)))))
    }

    val autonomousCommand: Command
        get() {
            val command = SequentialCommandGroup()
            val startingPose = startingPosition.pose
            command.addCommands(InstantCommand({ poseEstimation.resetPose(startingPose) }))
            return command
        }

    fun setTargetNode(targetNode: Node) {
        field.getObject("Node Position").pose = targetNode.nodePose
        this.targetNode = targetNode
    }

    companion object {
        // Dashboard
        val driveSettingsTab: ShuffleboardTab = Shuffleboard.getTab("Drive Settings")
        @JvmField
        val autoTab: ShuffleboardTab = Shuffleboard.getTab("Auto")
        @JvmField
        val swerveTab: ShuffleboardTab = Shuffleboard.getTab("Swerve")
        @JvmField
        val armTab: ShuffleboardTab = Shuffleboard.getTab("Arm")

        // Subsystems
        @JvmField
        val drivetrain = Drivetrain()
        @JvmField
        val arm = Arm()

        // Controllers
        val joystickLeft = Joystick(Constants.ControlConstants.JOYSTICK_RIGHT_PORT)
        @JvmField
        val joystickRight = Joystick(Constants.ControlConstants.JOYSTICK_LEFT_PORT)
        val controller = XboxController(Constants.ControlConstants.CONTROLLER_PORT)
        val buttonPanel = Joystick(Constants.ControlConstants.BUTTON_PANEL_PORT)

        // Pose Estimation
        @JvmField
        val poseEstimation = PoseEstimation()
        val drivePresetsChooser = SendableChooser<String>()
        @JvmField
        var field = Field2d()

        // RGB
        val lights = LightsTable()
    }
}