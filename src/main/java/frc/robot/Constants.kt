// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.revrobotics.CANSparkMax.IdleMode
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units

class Constants {
    object ControlConstants {
        const val JOYSTICK_RIGHT_PORT = 0
        const val JOYSTICK_LEFT_PORT = 1
        const val CONTROLLER_PORT = 2
        const val BUTTON_PANEL_PORT = 3
    }

    object DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        const val MAX_SPEED_METERS_PER_SECOND = 4.8
        const val MAX_ANGULAR_SPEED = 2 * Math.PI // radians per second

        // Chassis configuration
        @JvmField
        val TRACK_WIDTH = Units.inchesToMeters(22.5)

        // Distance between centers of right and left wheels on robot
        @JvmField
        val WHEEL_BASE = Units.inchesToMeters(24.5)

        // Distance between front and back wheels on robot
        @JvmField
        val DRIVE_KINEMATICS = SwerveDriveKinematics(
                Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2))

        // Angular offsets of the modules relative to the chassis in radians
        const val FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2
        const val FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0.0
        const val REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI
        const val REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2

        // Charge Station Constants
        @JvmField
        val CHARGE_STATION_TOLERANCE = Math.toRadians(11.0)

        // Delay between reading the gyro and using the value used to aproximate exact angle while spinning (0.02 is one loop)
        const val GYRO_READ_DELAY = 0.02

        // SPARK MAX CAN IDs
        const val FRONT_LEFT_DRIVING_CAN_ID = 10
        const val REAR_LEFT_DRIVING_CAN_ID = 12
        const val FRONT_RIGHT_DRIVING_CAN_ID = 14
        const val REAR_RIGHT_DRIVING_CAN_ID = 16
        const val FRONT_LEFT_TURNING_CAN_ID = 11
        const val REAR_LEFT_TURNING_CAN_ID = 13
        const val FRONT_RIGHT_TURNING_CAN_ID = 15
        const val REAR_RIGHT_TURNING_CAN_ID = 17
        const val GYRO_REVERSED = false
        @JvmField
        val GYRO_ROTATION = Rotation3d(0.0, 0.0, -Math.PI / 2)
        @JvmField
        val ODOMETRY_STD_DEV = VecBuilder.fill(0.05, 0.05, 0.01)
    }

    object Arm {
        const val HUMERUS_LENGTH = 1.010922
        const val PIVOT_HEIGHT = 1.162025
        const val PIVOT_FORWARD_OFFSET = 0.203391
        const val HIGH_CONE_SCORING_DIST = 1.46
        const val HIGH_CUBE_SCORING_DIST = 1.6
        const val MID_CONE_SCORING_DIST = 1.4
        const val MID_CUBE_SCORING_DIST = 1.5
        const val LOW_CONE_SCORING_DIST = 1.5
        const val LOW_CUBE_SCORING_DIST = 1.0
        @JvmField
        val RELATIVE_WRIST_POSE = Translation2d(0.0, -HUMERUS_LENGTH)
    }

    object Shoulder {
        const val MOTOR_1_ID = 5
        const val MOTOR_2_ID = 6
        const val GEAR_RATIO = 15.0 / 36.0
        const val KS = 0.23446
        const val KG = 0.66001
        const val KV = 2.07220
        const val KA = 0.19625
        const val KP = 1.0
        const val KI = 0.0
        const val KD = 0.03
        @JvmField
        val STOWED_ANGLE = Rotation2d.fromRadians(-0.109670)
        @JvmField
        val HIGH_CONE_ANGLE = Rotation2d.fromRadians(1.910583)
        @JvmField
        val HIGH_CUBE_ANGLE = Rotation2d.fromRadians(1.570526)
        @JvmField
        val MID_CONE_ANGLE = Rotation2d.fromRadians(1.867019)
        @JvmField
        val MID_CUBE_ANGLE = Rotation2d.fromRadians(1.282185)
        @JvmField
        val INTAKE_CONE_ANGLE = Rotation2d.fromRadians(1.082943)
        @JvmField
        val LOW_CUBE_ANGLE = Rotation2d.fromRadians(0.6)
        @JvmField
        val SLIDE_CONE_ANGLE = Rotation2d.fromRadians(0.873581)
        @JvmField
        val SLIDE_CUBE_ANGLE = Rotation2d.fromRadians(1.037839)
        @JvmField
        val TELLER_CONE_ANGLE = Rotation2d.fromRadians(2.111365)
        @JvmField
        val TELLER_CUBE_ANGLE = Rotation2d.fromRadians(1.649983)
        @JvmField
        val MAX_ANGLE = Rotation2d.fromRadians(2.3)
        @JvmField
        val TOLERANCE_ANGLE = Rotation2d.fromRadians(1.0)
        val FINISH_TOLERANCE = Units.degreesToRadians(0.0)
        @JvmField
        val TRAPEZOID_PROFILE_CONSTRAINTS = TrapezoidProfile.Constraints(8.0, 8.0)
    }

    object Wrist {
        const val ID = 7
        const val GEAR_RATIO = 1.0 / 75.0
        const val KP = 7.0
        const val KI = 0.1
        const val KD = 0.25
        const val KS = 0.14588
        const val KG = 0.13063
        const val KV = 1.4242
        const val KA = 0.052069
        @JvmField
        val STOWED_ANGLE = Rotation2d.fromRadians(0.0)
        @JvmField
        val HIGH_CONE_ANGLE = Rotation2d.fromDegrees(-18.185489)
        @JvmField
        val HIGH_CUBE_ANGLE = Rotation2d.fromRadians(0.966711)
        @JvmField
        val MID_CONE_ANGLE = Rotation2d.fromRadians(-0.725371)
        @JvmField
        val MID_CUBE_ANGLE = Rotation2d.fromRadians(0.966711)
        @JvmField
        val INTAKE_CONE_ANGLE = Rotation2d.fromDegrees(-56.414788)
        @JvmField
        val LOW_CUBE_ANGLE = Rotation2d.fromDegrees(41.957416)
        @JvmField
        val SLIDE_CONE_ANGLE = Rotation2d.fromDegrees(61.061158)
        @JvmField
        val SLIDE_CUBE_ANGLE = Rotation2d.fromDegrees(107.388460)
        @JvmField
        val TELLER_CONE_ANGLE = Rotation2d.fromDegrees(-41.560697)
        @JvmField
        val TELLER_CUBE_ANGLE = Rotation2d.fromDegrees(35.388460)
        @JvmField
        val MIN_SHOULDER_ANGLE = Rotation2d.fromRadians(0.709869)
        const val LIMIT_SWITCH = 0
        @JvmField
        val LIMIT_SWITCH_OFFSET = Rotation2d.fromDegrees(63.0)
        @JvmField
        val CONE_ANGLE = Rotation2d.fromDegrees(-40.0)
        @JvmField
        val CUBE_ANGLE = Rotation2d.fromDegrees(40.0)

        //Min Angle
        const val HORIZONTAL_TO_CORNER_ANGLE = 0.2985176246
        @JvmField
        val JOINT_TO_CORNER_DISTANCE = Units.inchesToMeters(14.0)
        const val CLEARANCE_HEIGHT = 1.0
    }

    object Rollers {
        const val ID = 8
        const val INTAKE_CONE = -.6
        const val INTAKE_CUBE = 0.5
        const val OUTTAKE_CONE = 0.5
        const val OUTTAKE_CUBE = -.6
    }

    object ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        const val DRIVING_MOTOR_PINION_TEETH = 13

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        const val TURNING_ENCODER_INVERTED = true

        // Calculations required for driving motor conversion factors and feed forward
        const val DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60
        val WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.0)
        @JvmField
        val WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        const val DRIVING_MOTOR_REDUCTION = 45.0 * 22 / (DRIVING_MOTOR_PINION_TEETH * 15)
        val DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS
                / DRIVING_MOTOR_REDUCTION)
        @JvmField
        val DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_CIRCUMFERENCE_METERS
                / DRIVING_MOTOR_REDUCTION // meters
                )
        @JvmField
        val DRIVING_ENCODER_VELOCITY_FACTOR = (WHEEL_CIRCUMFERENCE_METERS
                / DRIVING_MOTOR_REDUCTION) / 60.0 // meters per second
        const val TURNING_ENCODER_POSITION_FACTOR = 2 * Math.PI // radians
        const val TURNING_ENCODER_VELOCITY_FACTOR = 2 * Math.PI / 60.0 // radians per second
        const val TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0.0 // radians
        const val TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR // radians
        const val DRIVING_P = 0.04
        const val DRIVING_I = 0.0
        const val DRIVING_D = 0.0
        @JvmField
        val DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS
        const val DRIVING_MIN_OUTPUT = -1.0
        const val DRIVING_MAX_OUTPUT = 1.0
        const val TURNING_P = 2.0
        const val TURNING_I = 0.0
        const val TURNING_D = 0.0
        const val TURNING_FF = 0.0
        const val TURNING_MIN_OUTPUT = -1.0
        const val TURNING_MAX_OUTPUT = 1.0
        @JvmField
        val DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake
        @JvmField
        val TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake
        const val DRIVING_MOTOR_CURRENT_LIMIT = 50 // amps
        const val TURNING_MOTOR_CURRENT_LIMIT = 20 // amps
    }

    object AutoConstants {
        const val MAX_SPEED_METERS_PER_SECOND = 5.0
        const val MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 5.0
        const val MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 2
        const val MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI * 2
        const val P_TRANSLATION_PATH_CONTROLLER = 4.0
        const val P_THETA_PATH_CONTROLLER = 4.0
        const val P_TRANSLATION_POINT_CONTROLLER = 2.0
        const val P_THETA_POINT_CONTROLLER = 4.0
        const val TRANSLATION_TOLERANCE = 0.01
        @JvmField
        val THETA_TOLERANCE = Rotation2d.fromDegrees(1.0)

        // Constraint for the motion profiled robot angle controller
        val THETA_CONTROLLER_CONSTRAINTS = TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED)
        val NODE_HIGH_TRANSFORM = Transform2d(
                Translation2d(-1.0, 0.0),
                Rotation2d.fromRadians(Math.PI)
        )
    }

    object NeoMotorConstants {
        const val FREE_SPEED_RPM = 5676.0
    }

    object VisionConstants {
        // FIXME: actually measure these constants
        @JvmField
        val PHOTONVISION_TRANSFORM = Transform3d(
                Translation3d(0.205697, -0.244475, 0.267365),
                Rotation3d(0.0, Units.degreesToRadians(15.0), 0.0)
        )
        @JvmField
        val PHOTONVISION_STD_DEV = VecBuilder.fill(0.5, 0.5, 0.3)
        @JvmField
        val LIMELIGHT_STD_DEV = VecBuilder.fill(0.9, 0.9, 0.9)
        const val AMBIGUITY_FILTER = 0.05
    }

    object FieldConstants {
        @JvmField
        val fieldLength = Units.inchesToMeters(651.25)
        val fieldWidth = Units.inchesToMeters(315.5)
        val tapeWidth = Units.inchesToMeters(2.0)
        val aprilTagWidth = Units.inchesToMeters(6.0)
        val aprilTags = mapOf(
                1 to
                Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(42.19),
                        Units.inchesToMeters(18.22),
                        Rotation3d(0.0, 0.0, Math.PI)),
                2 to
                Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(108.19),
                        Units.inchesToMeters(18.22),
                        Rotation3d(0.0, 0.0, Math.PI)),
                3 to
                Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(174.19),  // FIRST's diagram has a typo (it says 147.19)
                        Units.inchesToMeters(18.22),
                        Rotation3d(0.0, 0.0, Math.PI)),
                4 to
                Pose3d(
                        Units.inchesToMeters(636.96),
                        Units.inchesToMeters(265.74),
                        Units.inchesToMeters(27.38),
                        Rotation3d(0.0, 0.0, Math.PI)),
                5 to
                Pose3d(
                        Units.inchesToMeters(14.25),
                        Units.inchesToMeters(265.74),
                        Units.inchesToMeters(27.38),
                        Rotation3d()),
                6 to
                Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(174.19),  // FIRST's diagram has a typo (it says 147.19)
                        Units.inchesToMeters(18.22),
                        Rotation3d()),
                7 to
                Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(108.19),
                        Units.inchesToMeters(18.22),
                        Rotation3d()),
                8 to
                Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(42.19),
                        Units.inchesToMeters(18.22),
                        Rotation3d()
                )
        )

        object Grids {
            @JvmField
            val GRID_BOUNDARIES = doubleArrayOf(0.0, 1.910, 3.586, 5.446649)

            // X layout
            val outerX = Units.inchesToMeters(54.25)
            val lowX = outerX - Units.inchesToMeters(14.25) / 2.0 // Centered when under cube nodes
            val midX = outerX - Units.inchesToMeters(22.75)
            val highX = outerX - Units.inchesToMeters(39.75)

            // Y layout
            const val nodeRowCount = 9
            val nodeFirstY = Units.inchesToMeters(20.19)
            val nodeSeparationY = Units.inchesToMeters(22.0)

            // Z layout
            val cubeEdgeHigh = Units.inchesToMeters(3.0)
            val highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh
            val midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh
            val highConeZ = Units.inchesToMeters(46.0)
            val midConeZ = Units.inchesToMeters(34.0)

            // Translations (all nodes in the same column/row have the same X/Y coordinate)
            @JvmField
            val lowTranslations = arrayOfNulls<Translation2d>(nodeRowCount)
            val low3dTranslations = arrayOfNulls<Translation3d>(nodeRowCount)
            @JvmField
            val midTranslations = arrayOfNulls<Translation2d>(nodeRowCount)
            val mid3dTranslations = arrayOfNulls<Translation3d>(nodeRowCount)
            @JvmField
            val highTranslations = arrayOfNulls<Translation2d>(nodeRowCount)
            val high3dTranslations = arrayOfNulls<Translation3d>(nodeRowCount)

            init {
                for (i in 0 until nodeRowCount) {
                    val isCube = i == 1 || i == 4 || i == 7
                    lowTranslations[i] = Translation2d(lowX, nodeFirstY + nodeSeparationY * i)
                    low3dTranslations[i] = Translation3d(lowX, nodeFirstY + nodeSeparationY * i, 0.0)
                    midTranslations[i] = Translation2d(midX, nodeFirstY + nodeSeparationY * i)
                    mid3dTranslations[i] = Translation3d(midX, nodeFirstY + nodeSeparationY * i, if (isCube) midCubeZ else midConeZ)
                    high3dTranslations[i] = Translation3d(
                            highX, nodeFirstY + nodeSeparationY * i, if (isCube) highCubeZ else highConeZ)
                    highTranslations[i] = Translation2d(highX, nodeFirstY + nodeSeparationY * i)
                }
            }
        }
    }
}