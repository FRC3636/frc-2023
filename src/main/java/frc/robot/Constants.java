// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Controls {
    public static final int JOYSTICK_RIGHT_PORT = 0;
    public static final int JOYSTICK_LEFT_PORT = 1;
  }

  public static class Drivetrain {
    public static final int MOTOR_RIGHT = 0;
    public static final int MOTOR_LEFT = 1;

    public static final int ENCODER_RIGHT_PORT_A = 0;
    public static final int ENCODER_RIGHT_PORT_B  = 1;
    public static final int ENCODER_LEFT_PORT_A  = 2;
    public static final int ENCODER_LEFT_PORT_B  = 3;

    public static final double TRACK_WIDTH = 0.54; // in meters

    public static final int PULSES_PER_REVOLUTION = 4096;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    // amy
    public static final double FEED_FORWARD_KS = -70.693;
    public static final double FEED_FORWARD_KV = 33.277;
    public static final double FEED_FORWARD_KA = 310.93;

    public static final double DRIVE_VELOCITY_KP = 1.8538;
  }

  public static class Robot {
    public static final Pose2d CAMERA_OFFSET = new Pose2d(Units.inchesToMeters(14), 0.0, new Rotation2d());
  }
}
