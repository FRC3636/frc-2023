package frc.robot.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.arm.Arm

class ArmHoldCommand(private val arm: Arm) : CommandBase() {
    init {
        addRequirements(arm)
    }

    override fun execute() {
        arm.runWithSetpoint(Arm.State.target.shoulderAngle, Rotation2d())
    }
}