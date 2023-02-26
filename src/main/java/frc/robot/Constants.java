// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import java.util.Map;

import static com.revrobotics.CANSparkMax.IdleMode;

public final class Constants {
    public static class ControlConstants {
        public static final int JOYSTICK_RIGHT_PORT = 0;
        public static final int JOYSTICK_LEFT_PORT = 1;
        public static final int CONTROLLER_PORT = 2;
    }

    public static class Drivetrain {
        public static final int MOTOR_RIGHT_1 = 1;
        public static final int MOTOR_RIGHT_2 = 2;
        public static final int MOTOR_LEFT_1 = 3;
        public static final int MOTOR_LEFT_2 = 4;


        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        public static final double SENSOR_UNITS_PER_REV = 2048;
        public static final double GEAR_RATIO = 10.71;
        public static final double SENSOR_UNITS_PER_METER =
                (SENSOR_UNITS_PER_REV * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;

        public static final double TRACK_WIDTH = 0.54; // in meters

        public static final int PULSES_PER_REVOLUTION = 4096;

        // amy
        public static final double FEED_FORWARD_KS = -70.693;
        public static final double FEED_FORWARD_KV = 33.277;
        public static final double FEED_FORWARD_KA = 310.93;

        public static final double DRIVE_VELOCITY_KP = 1.8538;
    }

    public static class Arm {
        public static final double HUMERUS_LENGTH = Units.inchesToMeters(40);

        public static final double PIVOT_HEIGHT = Units.inchesToMeters(45.6);
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
        public static final Rotation2d INTAKE_CONE = Rotation2d.fromRadians(1.111091);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromRadians(2.3);

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
        public static final Rotation2d INTAKE_CONE = Rotation2d.fromRadians(-0.679611);

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
