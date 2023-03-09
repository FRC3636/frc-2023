// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.RobotContainer
import frc.robot.subsystems.arm.Arm.State.GamePiece.Companion.fromLevelAndColumn
import frc.robot.utils.Node
import frc.robot.utils.Node.Column

class MoveNodeSelection(robot: RobotContainer, direction: MovementDirection?) : InstantCommand(Runnable {
    val currentTarget = robot.targetNode
    var column = currentTarget.column.ordinal
    var level = currentTarget.level.ordinal
    when (direction) {
        MovementDirection.Up -> {
            level += 1
        }

        MovementDirection.Down -> {
            level -= 1
        }

        MovementDirection.Left -> {
            column += 1
        }

        MovementDirection.Right -> {
            column -= 1
        }

        else -> {}
    }

    // wrap around if too high or too low
    if (column < 0) {
        column = maxColumn - 1
    } else if (column >= maxColumn) {
        column = 0
    }
    if (level < 0) {
        level = maxLevel - 1
    } else if (level >= maxLevel) {
        level = 0
    }

    // convert level and column back to a node object
    val levelObject = Node.Level.values()[level]
    val columnObject = Column.values()[column]
    val nodeType = fromLevelAndColumn(level, column)
    val newTarget = Node(nodeType, levelObject, columnObject)
    robot.setTargetNode(newTarget)
    println("New node selection -> (level %s, %s)".format(levelObject, columnObject))
}) {
    enum class MovementDirection {
        Up, Down, Left, Right
    }

    companion object {
        private val maxColumn = Column.values().size
        private val maxLevel = Node.Level.values().size
    }
}