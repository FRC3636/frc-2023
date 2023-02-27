// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import java.util.Map;

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
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
        // Distance between centers of right and left wheels on robot
        public static final double WHEEL_BASE = Units.inchesToMeters(24.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
        public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
        public static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
        public static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

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

        public static final Vector<N3> ODOMETRY_STD_DEV = VecBuilder.fill(0.05, 0.05, 0.001);
    }

    public static class Arm {
        public static final double HUMERUS_LENGTH = 1.010922;
        public static final double PIVOT_HEIGHT = 1.162025;
        public static final double PIVOT_FORWARD_OFFSET = 0.203391;

        public static final double MID_CONE_SCORING_DIST = 1.4;
        public static final double HIGH_CONE_SCORING_DIST = 1.46;

        public static final double MID_CUBE_SCORING_DIST = 1.5;
        public static final double HIGH_CUBE_SCORING_DIST = 1.6;

        public static final Translation2d RELATIVE_WRIST_POSE = new Translation2d(0, -HUMERUS_LENGTH);
    }

    public static class Shoulder {
        public static final int MOTOR_1_ID = 5;
        public static final int MOTOR_2_ID = 6;
        public static final double GEAR_RATIO = 15.0 / 36.0;

        public static final double KS = 0.23446;
        public static final double KG = 0.66001;
        public static final double KV = 2.07220;
        public static final double KA = 0.19625;

        public static final double KP = 1;
        public static final double KI = 0.0;
        public static final double KD = 0.03;

        public static final Rotation2d STOWED_ANGLE = Rotation2d.fromRadians(-0.143279);
        public static final Rotation2d HIGH_CONE_ANGLE = Rotation2d.fromRadians(2.178937);
        public static final Rotation2d HIGH_CUBE_ANGLE = Rotation2d.fromRadians(1.570526);
        public static final Rotation2d MID_CONE_ANGLE = Rotation2d.fromRadians(1.867019);
        public static final Rotation2d MID_CUBE_ANGLE = Rotation2d.fromRadians(1.282185);
        public static final Rotation2d INTAKE_CONE_ANGLE = Rotation2d.fromRadians(1.111091);
        public static final Rotation2d SLIDE_CONE_ANGLE = Rotation2d.fromRadians(0.873581);
        public static final Rotation2d SLIDE_CUBE_ANGLE = Rotation2d.fromRadians(1.037839);
        public static final Rotation2d TELLER_CONE_ANGLE = Rotation2d.fromRadians(2.111365);
        public static final Rotation2d TELLER_CUBE_ANGLE = Rotation2d.fromRadians(1.649983);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromRadians(2.3);
        public static final Rotation2d TOLERANCE_ANGLE = Rotation2d.fromRadians(1);

        public static final double FINISH_TOLERANCE = Units.degreesToRadians(0);

        public static final TrapezoidProfile.Constraints TRAPEZOID_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
    }

    public static class Wrist {
        public static final int ID = 7;

        public static final double GEAR_RATIO = 1.0 / 75.0;

        public static final double KP = 7;
        public static final double KI = 0.1;
        public static final double KD = 0.25;

        public static final double KS = 0.14588;
        public static final double KG = 0.13063;
        public static final double KV = 1.4242;
        public static final double KA = 0.052069;

        public static final Rotation2d STOWED_ANGLE = Rotation2d.fromRadians(0);
        public static final Rotation2d HIGH_CONE_ANGLE = Rotation2d.fromRadians(-0.457023);
        public static final Rotation2d HIGH_CUBE_ANGLE = Rotation2d.fromRadians(0.966711);
        public static final Rotation2d MID_CONE_ANGLE = Rotation2d.fromRadians(-0.725371);
        public static final Rotation2d MID_CUBE_ANGLE = Rotation2d.fromRadians(0.966711);
        public static final Rotation2d INTAKE_CONE_ANGLE = Rotation2d.fromRadians(-0.679611);
        public static final Rotation2d SLIDE_CONE_ANGLE = Rotation2d.fromDegrees(61.061158);
        public static final Rotation2d SLIDE_CUBE_ANGLE = Rotation2d.fromDegrees(107.388460);
        public static final Rotation2d TELLER_CONE_ANGLE = Rotation2d.fromDegrees(-41.560697);
        public static final Rotation2d TELLER_CUBE_ANGLE = Rotation2d.fromDegrees(35.388460);

        public static final Rotation2d MIN_SHOULDER_ANGLE = Rotation2d.fromRadians(0.709869);

        public static final int LIMIT_SWITCH = 1;
        public static final Rotation2d LIMIT_SWITCH_OFFSET = Rotation2d.fromDegrees(63);
        public static final Rotation2d CONE_ANGLE = Rotation2d.fromDegrees(-40);
        public static final Rotation2d CUBE_ANGLE = Rotation2d.fromDegrees(40);

        //Min Angle
        public static final double HORIZONTAL_TO_CORNER_ANGLE = 0.2985176246;
        public static final double JOINT_TO_CORNER_DISTANCE = Units.inchesToMeters(14);
        public static final double CLEARANCE_HEIGHT = 1;


    }

    public static class Rollers {
        public static final int ID = 8;
        public static final double INTAKE_CONE = -.6;
        public static final double INTAKE_CUBE = 0.5;
        public static final double OUTTAKE_CONE = 0.5;
        public static final double OUTTAKE_CUBE = -.6;

    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int DRIVING_MOTOR_PINION_TEETH = 13;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean TURNING_ENCODER_INVERTED = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
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

        public static final double TURNING_P = 1;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 0;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT = -1;
        public static final double TURNING_MAX_OUTPUT = 1;

        public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

        public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 5;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 5;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double PX_CONTROLLER = 1;
        public static final double P_THETA_CONTROLLER = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
    }

    public static final class NeoMotorConstants {
        public static final double FREE_SPEED_RPM = 5676;
    }

    public static class VisionConstants {
        // FIXME: actually measure these constants

        public static final Transform3d PHOTONVISION_TRANSFORM = new Transform3d(
                new Translation3d(0, 0, 0.1),
                new Rotation3d(0, Units.degreesToRadians(15), 0)
        );

        public static final Vector<N3> PHOTONVISION_STD_DEV = VecBuilder.fill(0.9, 0.9, 0.9);

        public static final Vector<N3> LIMELIGHT_STD_DEV = VecBuilder.fill(0.9, 0.9, 0.9);
    }

    public static class FieldConstants {
        public static final double fieldLength = Units.inchesToMeters(651.25);
        public static final double fieldWidth = Units.inchesToMeters(315.5);
        public static final double tapeWidth = Units.inchesToMeters(2.0);
        public static final double aprilTagWidth = Units.inchesToMeters(6.0);
        public static class Grids {
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
