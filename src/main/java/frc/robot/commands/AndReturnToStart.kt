package frc.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.poseestimation.PoseEstimation
import frc.robot.subsystems.drivetrain.Drivetrain

//Run the specified command, then return the robot to its position before the specified command was executed.
class AndReturnToStart(private val poseEstimation: PoseEstimation, private val drivetrain: Drivetrain, private val inner: Command) : Command {
    private var innerEnded = false
    private var start: Pose2d? = null
    private var returnToStart: NavigateToPoint? = null
    override fun initialize() {
        start = poseEstimation.estimatedPose
        innerEnded = false
        inner.initialize()
    }

    override fun execute() {
        if (!innerEnded) {
            if (!inner.isFinished) {
                inner.execute()
            } else {
                inner.end(false)
                innerEnded = true
                returnToStart = NavigateToPoint(drivetrain, poseEstimation, start)
                returnToStart!!.initialize()
                returnToStart!!.execute()
            }
        } else {
            returnToStart!!.execute()
        }
    }

    override fun isFinished(): Boolean {
        return innerEnded && returnToStart!!.isFinished
    }

    override fun end(interrupted: Boolean) {
        if (!innerEnded) {
            inner.end(interrupted)
        } else {
            returnToStart!!.end(interrupted)
        }
    }

    override fun getRequirements(): Set<Subsystem> {
        val requirements = inner.requirements
        requirements.add(drivetrain)
        return requirements
    }
}