// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
        public static final int SHOULDER_1_ID = 5;
        public static final int SHOULDER_2_ID = 6;
        public static final double SHOULDER_GEAR_RATIO = 1.0 / 2.0;
        public static final double SHOULDER_KS = 0.0;
        public static final double SHOULDER_KG = 0.0;
        public static final double SHOULDER_KV = 0.0;
        public static final double SHOULDER_KA = 0.0;
        public static final double SHOULDER_KP = 0.0;
        public static final double SHOULDER_KI = 0.0;
        public static final double SHOULDER_KD = 0.0;
        public static final double SHOULDER_STOWED_ANGLE = Units.degreesToRadians(-9);
        public static final double SHOULDER_HIGH_ANGLE = Units.degreesToRadians(96);
        public static final double SHOULDER_MID_ANGLE = Units.degreesToRadians(70);
        public static final double SHOULDER_LOW_ANGLE = Units.degreesToRadians(20);
    }

    public static class Wrist {
        public static final int WRIST_ID = 7;

        public static final double WRIST_GEAR_RATIO = 1.0 / 25.0;

        public static final double WRIST_KP = 0.0;
        public static final double WRIST_KI = 0.0;
        public static final double WRIST_KD = 0.0;

        public static final int WRIST_LIMIT_SWITCH = 1;
        public static final double WRIST_LIMIT_SWITCH_OFFSET = Units.degreesToRadians(-13);
    }

    public static class Claw {
        public static final int CLAW_ID = 8;
        public static final int CLAW_LIMIT_SWITCH = 0;
        public static final double CLAW_CONE_ANGLE =(25.6+54.4)/2;
        public static final double CLAW_CUBE_ANGLE = 25.6;
        public static final double CLAW_CLOSED_ANGLE = 54.4;
        public static final double CLAW_SPEED = 0.25;
        public static final double CLAW_CLAMP_THRESHOLD = 15.0;
        public static final double CLAW_GEAR_RATIO = 19.0 / (36.0 * 100.0);

        public static final int ROLLERS_ID = 9;
        public static final int ROLLER_OUT = 1;
        public static final int ROLLER_IN = -1;
        public static final int ROLLER_OFF = 0;
        public static final double ROLLER_SPEED = 0.5;
    }
    public static class Robot {

        public static final Pose2d CAMERA_OFFSET = new Pose2d(Units.inchesToMeters(14), 0.0, new Rotation2d());
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
