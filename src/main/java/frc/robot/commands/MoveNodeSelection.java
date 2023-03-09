// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.State.GamePiece;
import frc.robot.utils.Node;

public class MoveNodeSelection extends InstantCommand {
    private static final int maxColumn = Node.Column.values().length;
    private static final int maxLevel = Node.Level.values().length;

    /** Creates a new MoveNodeSelection. */
    public MoveNodeSelection(RobotContainer robot, final MovementDirection direction) {
        super(() -> {
            final Node currentTarget = robot.targetNode;
            int column = currentTarget.getColumn().ordinal();
            int level = currentTarget.getLevel().ordinal();

            switch (direction) {
                case Up: {
                    level += 1;
                    break;
                }
                case Down: {
                    level -= 1;
                    break;
                }
                case Left: {
                    column += 1;
                    break;
                }
                case Right: {
                    column -= 1;
                    break;
                }
            }

            // wrap around if too high or too low

            if (column < 0) {
                column = maxColumn - 1;
            } else if (column >= maxColumn) {
                column = 0;
            }

            if (level < 0) {
                level = maxLevel - 1;
            } else if (level >= maxLevel) {
                level = 0;
            }

            // convert level and column back to a node object

            final Node.Level levelObject = Node.Level.values()[level];
            final Node.Column columnObject = Node.Column.values()[column];

            final GamePiece nodeType = Arm.State.GamePiece.fromLevelAndColumn(level, column);
            final Node newTarget = new Node(nodeType, levelObject, columnObject);
            robot.setTargetNode(newTarget);

            System.out.println("New node selection -> (level %s, %s)".formatted(levelObject, columnObject));
        });
    }

    public enum MovementDirection {
        Up,
        Down,
        Left,
        Right;
    }
}
