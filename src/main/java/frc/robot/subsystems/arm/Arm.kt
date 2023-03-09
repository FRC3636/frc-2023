package frc.robot.subsystems.arm

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.RobotContainer
import frc.robot.commands.ArmMoveCommand
import frc.robot.utils.Node

class Arm : SubsystemBase() {
    private val shoulder: Shoulder = if (RobotBase.isSimulation()) SIMShoulder(this) else Shoulder(this)
    private val wrist: Wrist = if (RobotBase.isSimulation()) SIMWrist(this) else Wrist(this)
    private val rollers: Rollers = Rollers()
    private val arm = Mechanism2d(Units.inchesToMeters(28.0), Constants.Arm.PIVOT_HEIGHT + 0.5)
    private val pivot = arm.getRoot("pivot", Units.inchesToMeters(22.0), Constants.Arm.PIVOT_HEIGHT)
    private val humerus = pivot
            .append(MechanismLigament2d("humerus", Constants.Arm.HUMERUS_LENGTH, 0.0))
    private val manipulator = humerus
            .append(MechanismLigament2d("manipulator", Constants.Wrist.JOINT_TO_CORNER_DISTANCE, 0.0))
    private val setHumerus = pivot
            .append(MechanismLigament2d("set humerus", Constants.Arm.HUMERUS_LENGTH, 0.0))
    private val setManipulator = setHumerus
            .append(MechanismLigament2d("set manipulator", Constants.Wrist.JOINT_TO_CORNER_DISTANCE, 0.0))

    init {
        setHumerus.color = Color8Bit(0, 255, 255)
        setManipulator.color = Color8Bit(0, 100, 255)
        RobotContainer.armTab.addDoubleArray("Arm Position") { arm3dPose }
        RobotContainer.armTab.add("Arm", arm)
    }

    override fun periodic() {
        shoulder.periodic()
        wrist.periodic()
        rollers.periodic()
        humerus.setAngle(shoulderAngle!!.minus(Rotation2d.fromDegrees(90.0)))
        manipulator.setAngle(wrist.angle!!.plus(Rotation2d.fromDegrees(90.0)))
        setHumerus.setAngle(State.target.shoulderAngle.minus(Rotation2d.fromDegrees(90.0)))
        setManipulator.setAngle(State.target.wristAngle.plus(Rotation2d.fromDegrees(90.0))
                .minus(State.target.shoulderAngle))
    }

    val arm3dPose: DoubleArray
        get() {
            val shoulderPose = Pose3d(
                    Constants.Arm.PIVOT_FORWARD_OFFSET,
                    0.0,
                    Constants.Arm.PIVOT_HEIGHT,
                    Rotation3d(0.0, -shoulderAngle!!.radians, 0.0))
            val relativeWristOrigin = Constants.Arm.RELATIVE_WRIST_POSE.rotateBy(shoulderAngle)
            val wristPose = Pose3d(
                    Constants.Arm.PIVOT_FORWARD_OFFSET + relativeWristOrigin.x,
                    0.0,
                    Constants.Arm.PIVOT_HEIGHT + relativeWristOrigin.y,
                    Rotation3d(0.0, -wristAngle.radians, 0.0))
            return doubleArrayOf(
                    shoulderPose.x,
                    shoulderPose.y,
                    shoulderPose.z,
                    shoulderPose.rotation.quaternion.w,
                    shoulderPose.rotation.quaternion.x,
                    shoulderPose.rotation.quaternion.y,
                    shoulderPose.rotation.quaternion.z,
                    wristPose.x,
                    wristPose.y,
                    wristPose.z,
                    wristPose.rotation.quaternion.w,
                    wristPose.rotation.quaternion.x,
                    wristPose.rotation.quaternion.y,
                    wristPose.rotation.quaternion.z)
        }

    val shoulderAngle: Rotation2d
        get() = shoulder.angle
    val shoulderVelocity: Rotation2d
        get() = shoulder.velocity

    fun runWithSetpoint(shoulderPosition: Rotation2d, velocity: Rotation2d) {
        shoulder.runWithSetpoint(shoulderPosition, velocity, Rotation2d())
        wrist.followShoulderWithVelocity(velocity.times(-1.0))
    }

    val wristAngle: Rotation2d
        get() = shoulderAngle.plus(wrist.angle)

    enum class State {
        Teller(Constants.Shoulder.TELLER_CONE_ANGLE, Constants.Shoulder.TELLER_CUBE_ANGLE,
                Constants.Wrist.TELLER_CONE_ANGLE, Constants.Wrist.TELLER_CUBE_ANGLE),
        Slide(Constants.Shoulder.SLIDE_CONE_ANGLE, Constants.Shoulder.SLIDE_CUBE_ANGLE,
                Constants.Wrist.SLIDE_CONE_ANGLE, Constants.Wrist.SLIDE_CUBE_ANGLE),
        High(Constants.Shoulder.HIGH_CONE_ANGLE, Constants.Shoulder.HIGH_CUBE_ANGLE, Constants.Wrist.HIGH_CONE_ANGLE,
                Constants.Wrist.HIGH_CUBE_ANGLE),
        Mid(Constants.Shoulder.MID_CONE_ANGLE, Constants.Shoulder.MID_CUBE_ANGLE, Constants.Wrist.MID_CONE_ANGLE,
                Constants.Wrist.MID_CUBE_ANGLE),
        Low(Constants.Shoulder.INTAKE_CONE_ANGLE, Constants.Shoulder.LOW_CUBE_ANGLE, Constants.Wrist.INTAKE_CONE_ANGLE,
                Constants.Wrist.LOW_CUBE_ANGLE),
        Stowed(Constants.Shoulder.STOWED_ANGLE, Constants.Shoulder.STOWED_ANGLE, Constants.Wrist.STOWED_ANGLE,
                Constants.Wrist.CUBE_ANGLE);

        private val shoulderCubeAngle: Rotation2d
        private val shoulderConeAngle: Rotation2d
        private val wristConeAngle: Rotation2d
        private val wristCubeAngle: Rotation2d
        private var shoulderConeOffset = Rotation2d()
        private var shoulderCubeOffset = Rotation2d()
        private var wristConeOffset = Rotation2d()
        private var wristCubeOffset = Rotation2d()

        constructor(shoulderConeAngle: Rotation2d, shoulderCubeAngle: Rotation2d, wristConeAngle: Rotation2d,
                    wristCubeAngle: Rotation2d) {
            this.shoulderCubeAngle = shoulderCubeAngle
            this.shoulderConeAngle = shoulderConeAngle
            this.wristConeAngle = wristConeAngle
            this.wristCubeAngle = wristCubeAngle
        }

        constructor(shoulderAngle: Rotation2d) {
            shoulderCubeAngle = shoulderAngle
            shoulderConeAngle = shoulderAngle
            wristConeAngle = defaultWristAngle(GamePiece.Cone)
            wristCubeAngle = defaultWristAngle(GamePiece.Cube)
        }

        val shoulderAngle: Rotation2d
            get() = if (gamePiece == GamePiece.Cone) shoulderConeAngle.plus(shoulderConeOffset) else shoulderCubeAngle.plus(shoulderCubeOffset)
        val wristAngle: Rotation2d
            get() {
                if (this == Stowed && rollerState === Rollers.State.Off) {
                    return Constants.Wrist.LIMIT_SWITCH_OFFSET
                }
                return if (gamePiece == GamePiece.Cone) wristConeAngle.plus(wristConeOffset) else wristCubeAngle.plus(wristCubeOffset)
            }

        fun defaultWristAngle(gamePiece: GamePiece): Rotation2d {
            return if (gamePiece == GamePiece.Cone) Constants.Wrist.CONE_ANGLE else Constants.Wrist.CUBE_ANGLE
        }

        enum class GamePiece {
            Cone, Cube;

            companion object {
                fun fromNodeId(nodeId: Int): GamePiece {
                    return if (nodeId % 3 == 1) Cube else Cone
                }

                @JvmStatic
                fun fromLevelAndColumn(level: Int, column: Int): GamePiece {
                    val nodeId = level * 3 + column
                    return fromNodeId(nodeId)
                }
            }
        }

        companion object {
            var target = Stowed
                set(value) {
                    field = value
                    ArmMoveCommand(RobotContainer.arm).schedule()
                }

            private var gamePiece = GamePiece.Cube
            private var rollerState = Rollers.State.Off
            fun moveShoulderOffset(difference: Rotation2d) {
                when (gamePiece) {
                    GamePiece.Cone -> target.shoulderConeOffset = Rotation2d
                            .fromRadians(target.shoulderConeOffset.radians + difference.radians)

                    GamePiece.Cube -> target.shoulderCubeOffset = Rotation2d
                            .fromRadians(target.shoulderCubeOffset.radians + difference.radians)
                }
            }

            fun moveWristOffset(difference: Rotation2d) {
                when (gamePiece) {
                    GamePiece.Cone -> target.wristConeOffset = Rotation2d
                            .fromRadians(target.wristConeOffset.radians + difference.radians)

                    GamePiece.Cube -> target.wristCubeOffset = Rotation2d
                            .fromRadians(target.wristCubeOffset.radians + difference.radians)
                }
            }

            fun resetOffset() {
                when (gamePiece) {
                    GamePiece.Cone -> {
                        target.wristConeOffset = Rotation2d()
                        target.shoulderConeOffset = Rotation2d()
                    }

                    GamePiece.Cube -> {
                        target.wristCubeOffset = Rotation2d()
                        target.shoulderCubeOffset = Rotation2d()
                    }
                }
            }

            @JvmStatic
            fun setTargetFromNode(node: Node) {
                gamePiece = node.nodeType
                target = when (node.level) {
                    Node.Level.Low -> Low
                    Node.Level.Mid -> Mid
                    Node.Level.High -> High
                }
                ArmMoveCommand(RobotContainer.arm).schedule()
            }

            fun getGamePiece(): GamePiece {
                return gamePiece
            }

            fun setGamePiece(gamePiece: GamePiece) {
                Companion.gamePiece = gamePiece
                ArmMoveCommand(RobotContainer.arm).schedule()
            }

            @JvmStatic
            fun setRollerState(state: Rollers.State) {
                rollerState = state
                ArmMoveCommand(RobotContainer.arm).schedule()
            }

            val rollerSpeed: Double
                get() = if (gamePiece == GamePiece.Cone) rollerState.coneSpeed else rollerState.cubeSpeed

            fun getPresets(value: State): DoubleArray {
                return doubleArrayOf(
                        value.shoulderConeAngle.plus(value.shoulderConeOffset).radians,
                        value.shoulderCubeAngle.plus(value.shoulderCubeOffset).radians,
                        value.wristConeAngle.plus(value.wristConeOffset).radians,
                        value.wristCubeAngle.plus(value.wristCubeOffset).radians)
            }
        }
    }
}