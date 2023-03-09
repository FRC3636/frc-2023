package frc.robot.commands.alignment

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.poseestimation.PoseEstimation
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.utils.Node
import java.util.function.Supplier

//Moves the robot to the node returned by the specified node supplier.
class AlignToSelectedNode(drivetrain: Drivetrain, poseEstimation: PoseEstimation, targetNode: Supplier<Node>) : CommandBase() {
    private var innerCommand: AlignToNode
    private val drivetrain: Drivetrain
    private val poseEstimation: PoseEstimation
    private val targetNode: Supplier<Node>

    init {
        innerCommand = AlignToNode(drivetrain, poseEstimation, Node(0))
        this.drivetrain = drivetrain
        this.poseEstimation = poseEstimation
        this.targetNode = targetNode
    }

    override fun execute() {
        innerCommand.execute()
    }

    override fun initialize() {
        innerCommand = AlignToNode(drivetrain, poseEstimation, targetNode.get())
        innerCommand.initialize()
    }

    override fun end(terminated: Boolean) {
        innerCommand.end(terminated)
    }

    override fun isFinished(): Boolean {
        return innerCommand.isFinished
    }
}