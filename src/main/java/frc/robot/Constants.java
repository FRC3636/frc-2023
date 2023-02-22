// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import java.util.Map;

public final class Constants {
    public static class Controls {
        public static final int JOYSTICK_LEFT_PORT = 0;
        public static final int JOYSTICK_RIGHT_PORT = 1;
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

        public static final double STOWED_ANGLE = -0.143279;
        public static final double HIGH_CONE_ANGLE = 2.178937;
        public static final double HIGH_CUBE_ANGLE = 1.570526;
        public static final double MID_CONE_ANGLE = 1.867019;
        public static final double MID_CUBE_ANGLE = 1.282185;
        public static final double INTAKE_CONE = 1.111091;
        public static final double MAX_ANGLE = 2.3;

        public static final double FINISH_TOLERANCE = Units.degreesToRadians(0);

        public static final TrapezoidProfile.Constraints TRAPEZOID_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 6);
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

        public static final double STOWED_ANGLE = -0.143279;
        public static final double HIGH_CONE_ANGLE = -0.457023;
        public static final double HIGH_CUBE_ANGLE = 0.966711;
        public static final double MID_CONE_ANGLE = -0.725371;
        public static final double MID_CUBE_ANGLE = 0.966711;
        public static final double INTAKE_CONE = -0.679611;

        public static final double MIN_SHOULDER_ANGLE = 0.709869;

        public static final int LIMIT_SWITCH = 1;
        public static final double LIMIT_SWITCH_OFFSET = Units.degreesToRadians(63);
        public static final double CONE_ANGLE = Units.degreesToRadians(-40);
        public static final double CUBE_ANGLE = Units.degreesToRadians(40);
    }

    public static class Rollers {
        public static final int ID = 8;
        public static final int INTAKE_CONE = -1;
        public static final int INTAKE_CUBE = 1;
        public static final double SPEED = .6;
    }

    public static class FieldConstants {
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
                                new Rotation3d()));
    }
}
