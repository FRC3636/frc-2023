// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.PieceDependent;

import java.util.Map;
import java.util.function.BiFunction;

public final class Constants {
    public static class ControlConstants {
        public static final int JOYSTICK_RIGHT_PORT = 0;
        public static final int JOYSTICK_LEFT_PORT = 1;
        public static final int CONTROLLER_PORT = 2;

    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

        // Chassis configuration
        // Distance between centers of right and left wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
        // Distance between front and back wheels on robot
        public static final double WHEEL_BASE = Units.inchesToMeters(24.5);

        public static final Translation2d[] MODULE_POSITIONS = new Translation2d[]{
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
        };
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(MODULE_POSITIONS);

        public static final Rotation2d[] MODULE_ROTATIONS = new Rotation2d[]{
                Rotation2d.fromDegrees(-90),
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(180),
                Rotation2d.fromDegrees(90)
        };

        // Charge Station Constants
        public static final Rotation2d CHARGE_TIPPING_ANGLE = Rotation2d.fromDegrees(11);
        public static final Rotation2d CHARGE_TOLERANCE = Rotation2d.fromDegrees(2);
        public static final Rotation2d CHARGE_ANGULAR_VELOCITY_TOLERANCE = Rotation2d.fromDegrees(-5 * Robot.kDefaultPeriod);
        public static final double CHARGE_MAX_SPEED = 0.65;
        public static final double CHARGE_REDUCED_SPEED = 0.3;
        public static final double CHARGE_OSCILLATION_COEFFICIENT = 0.6;

        // Delay between reading the gyro and using the value used to aproximate exact angle while spinning (0.02 is one loop)
        public static final double GYRO_READ_DELAY = 0.02;

        // SPARK MAX CAN IDs
        public static final int FRONT_LEFT_DRIVING_CAN_ID = 10;
        public static final int REAR_LEFT_DRIVING_CAN_ID = 12;
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 14;
        public static final int REAR_RIGHT_DRIVING_CAN_ID = 16;

        public static final int FRONT_LEFT_TURNING_CAN_ID = 11;
        public static final int REAR_LEFT_TURNING_CAN_ID = 13;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 15;
        public static final int REAR_RIGHT_TURNING_CAN_ID = 17;

        public static final boolean GYRO_REVERSED = false;
        public static final Rotation3d GYRO_ROTATION = new Rotation3d(0, 0, -Math.PI / 2);

        public static final Vector<N3> ODOMETRY_STD_DEV = VecBuilder.fill(0.02, 0.02, 0.005);

        public static final double CARPET_BIAS = 0.05;

        public static final double DEADZONE = 0.1;
    }

    public static class Arm {
        public static final double HUMERUS_LENGTH = 0.969432;
        public static final double MANIPULATOR_LENGTH = 0.32;
        public static final double PIVOT_HEIGHT = 1.15;
        public static final double PIVOT_FORWARD_OFFSET = 0.203391;

        public static final double HIGH_CONE_SCORING_DIST = Units.inchesToMeters(63);
        public static final double HIGH_CUBE_SCORING_DIST = Units.inchesToMeters(60);
        public static final double MID_CONE_SCORING_DIST = 1.65;
        public static final double MID_CUBE_SCORING_DIST = Units.inchesToMeters(61);

        public static final double LOW_CONE_SCORING_DIST = 1.5;
        public static final double LOW_CUBE_SCORING_DIST = 1;

        public static final double RAISING_BUFFER_TIME = 1;
        public static final double INTAKING_BUFFER_TIME = 1;

        public static final double SAFE_RAISING_DISTANCE = 2.5;
        public static final double AUTO_RAISING_DISTANCE = 4;

        public static final Translation2d RELATIVE_WRIST_POSE = new Translation2d(0, -HUMERUS_LENGTH);

        public static final double STOWED_CUBE_HEIGHT = 0.27;
        public static final double STOWED_CONE_HEIGHT = 0.07;
        public static final double HIGH_CONE_HEIGHT = 1.376;
        public static final double HIGH_CUBE_HEIGHT = 1.33456;
        public static final double MID_CONE_HEIGHT = 1.04;
        public static final double MID_CUBE_HEIGHT = 1.04;
        public static final double LOW_CONE_HEIGHT = 0.35;
        public static final double LOW_CUBE_HEIGHT = 0.5;
        public static final double SLIDE_CONE_HEIGHT = 0.71;
        public static final double SLIDE_CUBE_HEIGHT = 0.90;
        public static final double TELLER_CONE_HEIGHT = 1.25;
        public static final double TELLER_CUBE_HEIGHT = 1.27;
    }

    public static class Shoulder {
        public static final int MOTOR_1_ID = 1;
        public static final int MOTOR_2_ID = 2;
        public static final double GEAR_RATIO = 15.0 / 36.0;

        public static final double KS = 0.23315;
        public static final double KG = 0.63300;
        public static final double KV = 2.14650;
        public static final double KA = 0.21209;

        public static final double DYNAMIC_KP = 1;
        public static final double STATIC_KP = 2;
        public static final double KI = 0.0;
        public static final double KD = 0.03;

        public static final Rotation2d STOWED_ANGLE = Rotation2d.fromRadians(-0.133);

        public static final Rotation2d MAX_ANGLE = Rotation2d.fromRadians(Math.PI);

        public static final TrapezoidProfile.Constraints TRAPEZOID_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
    }

    public static class Wrist {
        public static final int ID = 3;
        public static final int LIMIT_SWITCH = 0;

        public static final double GEAR_RATIO = 1.0 / 75.0;

        public static final double KP = 8;
        public static final double KI = 0.2;
        public static final double KD = 0.1;

        public static final double KS = 0.14588;
        public static final double KG = 0.13063;
        public static final double KV = 1.4242;
        public static final double KA = 0.052069;


        //Min Angle
        public static final Rotation2d ABSOLUTE_ENCODER_WRAPPING_ANGLE = Rotation2d.fromDegrees(70);
        public static final Rotation2d STOWED_WRIST_ANGLE = Rotation2d.fromDegrees(45);

        // Intaking Angles
        public static final Rotation2d STANDING_CONE_INTAKE_ANGLE = Rotation2d.fromDegrees(-60);
        public static final Rotation2d CUBE_INTAKE_ANGLE = Rotation2d.fromDegrees(30);
        public static final Rotation2d TIPPED_CONE_ANGLE = Rotation2d.fromRadians(-0.3);


        // Scoring Angles
        public static final Rotation2d SCORING_CONE_ANGLE = Rotation2d.fromRadians(-0.4);
        public static final Rotation2d SCORING_CUBE_ANGLE = Rotation2d.fromDegrees(90);

        // Slide
        public static final Rotation2d SLIDE_CONE_ANGLE = Rotation2d.fromRadians(0.960988);
        public static final Rotation2d SLIDE_CUBE_ANGLE = Rotation2d.fromDegrees(107.388460);

        public static final double HORIZONTAL_TO_CORNER_ANGLE = 0.2985176246;
        public static final double JOINT_TO_CORNER_DISTANCE = Units.inchesToMeters(14);
        public static final Rotation2d ENCODER_INTERPOLATION_SPEED = Rotation2d.fromDegrees(Robot.kDefaultPeriod);
    }

    public static class Rollers {
        public static final int ID = 4;
        public static final double INTAKE_CONE = -.6;
        public static final double INTAKE_CUBE = 0.5;
        public static final double OUTTAKE_CONE = 0.5;
        public static final double OUTTAKE_CUBE = -.6;
        public static final double HOLDING_PIECE_VELOCITY = 250;

        public static final int ECHO_CHANNEL = 2;
        public static final int PING_CHANNEL = 3;

        public static final double CONE_OFFSET = 0.00;
        public static final double CONE_CENTER_DISTANCE = Units.inchesToMeters(5.5);

    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int DRIVING_MOTOR_PINION_TEETH = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean TURNING_ENCODER_INVERTED = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_ENCODER_POSITION_FACTOR = WHEEL_CIRCUMFERENCE_METERS
                / DRIVING_MOTOR_REDUCTION; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = (WHEEL_CIRCUMFERENCE_METERS
                / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

        public static final double DRIVING_P = 0.04;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0;
        public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
        public static final double DRIVING_MIN_OUTPUT = -1;
        public static final double DRIVING_MAX_OUTPUT = 1;

        public static final double TURNING_P = 2;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 0;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT = -1;
        public static final double TURNING_MAX_OUTPUT = 1;

        public static final CANSparkMax.IdleMode DRIVING_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
        public static final CANSparkMax.IdleMode TURNING_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;

        public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps
    }

    public static final class AutoConstants {
        public static final String DEFAULT_PROGRAM = "score cube closest low cube;";

        public static final double MAX_SPEED_METERS_PER_SECOND = 8;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;

        public static final double P_TRANSLATION_CONTROLLER = 4;
        public static final double I_TRANSLATION_CONTROLLER = 0;
        public static final double D_TRANSLATION_CONTROLLER = 0.3;
        public static final double P_THETA_CONTROLLER = 6;

        public static final double TRANSLATION_TOLERANCE = 0.02;
        public static final Rotation2d THETA_TOLERANCE = Rotation2d.fromDegrees(1);

        public static final PieceDependent<Double> INTAKE_OFFSET = (piece) -> .5;

        public static final Pose2d BALANCE_STARTING_POINT_ALLIANCE_RELATIVE = new Pose2d(new Translation2d(5, 2.65), Rotation2d.fromRadians(Math.PI));
        public static final Pose2d BALANCE_LEAVE_COMMUNITY_POINT_ALLIANCE_RELATIVE = new Pose2d(new Translation2d(6.0, 2.65), Rotation2d.fromRadians(Math.PI));
        public static final double NODE_ALIGNMENT_CONTROL_HANDLE_LENGTH = 1;
    }

    public static final class NeoMotorConstants {
        public static final double FREE_SPEED_RPM = 5676;
    }

    public static class VisionConstants {
        // FIXME: actually measure these constants

        public static final Transform3d BACK_CAM_TRANSFORM = new Transform3d(
                new Translation3d(-0.127869, 0.060930, -0.463550),
                new Rotation3d(0, Units.degreesToRadians(15), Math.PI)
        );

        public static final Transform3d LEFT_CAM_TRANSFORM = new Transform3d(
                new Translation3d(.243, 0.229, -0.241),
                new Rotation3d(0, Units.degreesToRadians(15), 0)
        );

        public static final BiFunction<Double, Integer, Vector<N3>> PHOTON_VISION_STD_DEV =
                (distance, count) -> {
                    double distanceMultiplier = Math.pow(distance - ((count - 1) * 2), 2);
                    double translationalStdDev = (0.05 / (count)) * distanceMultiplier + 0.05;
                    double rotationalStdDev = 0.2 * distanceMultiplier + 0.1;
                    return VecBuilder.fill(
                            translationalStdDev,
                            translationalStdDev,
                            rotationalStdDev
                    );
                };

        public static final Vector<N3> LIMELIGHT_STD_DEV = VecBuilder.fill(0.9, 0.9, 0.9);

        public static final double AMBIGUITY_FILTER = 0.3;
        public static final double DISTANCE_FILTER = FieldConstants.fieldLength / 2;
    }

    public static class FieldConstants {
        public static final double fieldLength = 16.542;
        public static final double fieldWidth = 8.0137;
        public static final double tapeWidth = Units.inchesToMeters(2.0);
        public static final double aprilTagWidth = Units.inchesToMeters(6.0);

        public static final double PRESET_PIECE_X = 7;
        public static final double[] PRESET_PIECE_Y = new double[]{0.92, 2.14, 3.36, 4.57};

        public static class Grids {

            public static final double[] GRID_BOUNDARIES = new double[]{0, 1.910, 3.586, 5.446649};

            // X layout
            public static final double outerX = Units.inchesToMeters(54.25);
            public static final double lowX =
                    outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube nodes
            public static final double midX = outerX - Units.inchesToMeters(22.75);
            public static final double highX = outerX - Units.inchesToMeters(39.75);

            // Y layout
            public static final int nodeRowCount = 9;
            public static final double nodeFirstY = Units.inchesToMeters(20.19);
            public static final double nodeSeparationY = Units.inchesToMeters(22.0);

            // Z layout
            public static final double cubeEdgeHigh = Units.inchesToMeters(3.0);
            public static final double highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh;
            public static final double midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh;
            public static final double highConeZ = Units.inchesToMeters(46.0);
            public static final double midConeZ = Units.inchesToMeters(34.0);

            // Translations (all nodes in the same column/row have the same X/Y coordinate)
            public static final Translation2d[] lowTranslations = new Translation2d[nodeRowCount];
            public static final Translation3d[] low3dTranslations = new Translation3d[nodeRowCount];
            public static final Translation2d[] midTranslations = new Translation2d[nodeRowCount];
            public static final Translation3d[] mid3dTranslations = new Translation3d[nodeRowCount];
            public static final Translation2d[] highTranslations = new Translation2d[nodeRowCount];
            public static final Translation3d[] high3dTranslations = new Translation3d[nodeRowCount];

            static {
                for (int i = 0; i < nodeRowCount; i++) {
                    boolean isCube = i == 1 || i == 4 || i == 7;
                    lowTranslations[i] = new Translation2d(lowX, nodeFirstY + nodeSeparationY * i);
                    low3dTranslations[i] = new Translation3d(lowX, nodeFirstY + nodeSeparationY * i, 0.0);
                    midTranslations[i] = new Translation2d(midX, nodeFirstY + nodeSeparationY * i);
                    mid3dTranslations[i] =
                            new Translation3d(midX, nodeFirstY + nodeSeparationY * i, isCube ? midCubeZ : midConeZ);
                    high3dTranslations[i] =
                            new Translation3d(
                                    highX, nodeFirstY + nodeSeparationY * i, isCube ? highCubeZ : highConeZ);
                    highTranslations[i] = new Translation2d(highX, nodeFirstY + nodeSeparationY * i);
                }
            }
        }

        public static final Map<Integer, Pose3d> aprilTags =
                Map.of(
                        1,
                        new Pose3d(
                                Units.inchesToMeters(610.77),
                                Units.inchesToMeters(42.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d(0.0, 0.0, Math.PI)),
                        2,
                        new Pose3d(
                                Units.inchesToMeters(610.77),
                                Units.inchesToMeters(108.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d(0.0, 0.0, Math.PI)),
                        3,
                        new Pose3d(
                                Units.inchesToMeters(610.77),
                                Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                                Units.inchesToMeters(18.22),
                                new Rotation3d(0.0, 0.0, Math.PI)),
                        4,
                        new Pose3d(
                                Units.inchesToMeters(636.96),
                                Units.inchesToMeters(265.74),
                                Units.inchesToMeters(27.38),
                                new Rotation3d(0.0, 0.0, Math.PI)),
                        5,
                        new Pose3d(
                                Units.inchesToMeters(14.25),
                                Units.inchesToMeters(265.74),
                                Units.inchesToMeters(27.38),
                                new Rotation3d()),
                        6,
                        new Pose3d(
                                Units.inchesToMeters(40.45),
                                Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                                Units.inchesToMeters(18.22),
                                new Rotation3d()),
                        7,
                        new Pose3d(
                                Units.inchesToMeters(40.45),
                                Units.inchesToMeters(108.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d()),
                        8,
                        new Pose3d(
                                Units.inchesToMeters(40.45),
                                Units.inchesToMeters(42.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d()
                        )
                );
    }
}
