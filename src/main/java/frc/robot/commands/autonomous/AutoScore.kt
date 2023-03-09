package frc.robot.commands.autonomous

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.commands.alignment.AlignToSelectedNode
import frc.robot.poseestimation.PoseEstimation
import frc.robot.subsystems.arm.Arm.State.Companion.setRollerState
import frc.robot.subsystems.arm.Arm.State.Companion.setTargetFromNode
import frc.robot.subsystems.arm.Rollers
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.utils.Node
import java.util.function.Supplier

//Move to a node, position the arm and outtake the current game piece
class AutoScore(drivetrain: Drivetrain, poseEstimation: PoseEstimation, targetNode: Supplier<Node>) : SequentialCommandGroup(
        InstantCommand({ setTargetFromNode(targetNode.get()) }),
        AlignToSelectedNode(drivetrain, poseEstimation, targetNode),
        InstantCommand({ setRollerState(Rollers.State.Outtake) }),
        WaitCommand(0.5),
        InstantCommand({ setRollerState(Rollers.State.Off) })
)