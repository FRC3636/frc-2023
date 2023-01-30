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

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Controls {
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
        public static final int SHOULDER_ID = 5;
        public static final int CLAW_ID = 6;
        public static final int ROLLERS_ID = 7;

        public static final double SHOULDER_GEAR_RATIO = 1.0 / 192.0;

        public static final int POTENTIOMETER_PORT = 0;
        public static final double POTENTIOMETER_RANGE = 1; //TODO Measure
        public static final double POTENTIOMETER_OFFSET = 0; //TODO Measure

        public static final double ARM_HIGH_ANGLE = 90;
        public static final double ARM_MID_ANGLE = 70;
        public static final double ARM_LOW_ANGLE = 20;
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
