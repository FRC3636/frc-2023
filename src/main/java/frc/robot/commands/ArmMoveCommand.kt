package frc.robot.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.arm.Arm

class ArmMoveCommand(private val arm: Arm) : CommandBase() {
    private val timer = Timer()
    private var profile: TrapezoidProfile? = null

    init {
        addRequirements(arm)
    }

    override fun initialize() {
        SmartDashboard.putNumber("Shoulder Goal Position", Arm.State.target.shoulderAngle.radians)
        profile = TrapezoidProfile(
                Constants.Shoulder.TRAPEZOID_PROFILE_CONSTRAINTS,
                TrapezoidProfile.State(Arm.State.target.shoulderAngle.radians, 0.0),
                TrapezoidProfile.State(arm.shoulderAngle.radians, arm.shoulderVelocity.radians)
        )
        timer.reset()
        timer.start()
    }

    override fun execute() {
        val state = profile!!.calculate(timer.get())
        SmartDashboard.putNumber("Shoulder State Goal Position", state.position)
        arm.runWithSetpoint(Rotation2d.fromRadians(state.position), Rotation2d.fromRadians(state.velocity))
    }

    override fun isFinished(): Boolean {
        return profile!!.isFinished(timer.get())
    }
}