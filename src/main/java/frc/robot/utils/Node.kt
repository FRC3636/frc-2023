package frc.robot.utils

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.subsystems.arm.Arm.State.GamePiece
import frc.robot.utils.AllianceUtils.allianceToField
import frc.robot.utils.AllianceUtils.isBlue

class Node(val nodeType: GamePiece, val level: Level, val column: Column) {
    constructor(node: Int) : this(
            GamePiece.fromNodeId(node),
            Level.values()[node / 3],
            Column.values()[node % 3]) {
    }

    val robotOffset: Transform2d
        get() {
            var x = 0.0
            x = when (level) {
                Level.High -> if (nodeType == GamePiece.Cone) Constants.Arm.HIGH_CONE_SCORING_DIST else Constants.Arm.HIGH_CUBE_SCORING_DIST
                Level.Mid -> if (nodeType == GamePiece.Cone) Constants.Arm.MID_CONE_SCORING_DIST else Constants.Arm.MID_CUBE_SCORING_DIST
                Level.Low -> if (nodeType == GamePiece.Cone) Constants.Arm.LOW_CONE_SCORING_DIST else Constants.Arm.LOW_CUBE_SCORING_DIST
            }
            return Transform2d(Translation2d(if (isBlue) -x else x, 0.0),
                    Rotation2d(if (isBlue) 0.0 else Math.PI))
        }
    val nodePose: Pose2d?
        get() {
            val robotPose = RobotContainer.poseEstimation.estimatedPose
            val grid = if (robotPose.y > Constants.FieldConstants.Grids.GRID_BOUNDARIES[1]) if (robotPose.y > Constants.FieldConstants.Grids.GRID_BOUNDARIES[2]) 2 else 1 else 0
            val nodes: Array<Translation2d?> = when (level) {
                Level.High -> Constants.FieldConstants.Grids.highTranslations
                Level.Mid -> Constants.FieldConstants.Grids.midTranslations
                Level.Low -> Constants.FieldConstants.Grids.lowTranslations
            }
            return allianceToField(Pose2d(nodes[grid * 3 + column.index], Rotation2d(Math.PI)))
        }

    enum class Level {
        High, Mid, Low
    }

    enum class Column(private val blueIndex: Int) {
        LeftCone(2), Cube(1), RightCone(0);

        val index: Int
            get() = if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) blueIndex else 2 - blueIndex
    }
}