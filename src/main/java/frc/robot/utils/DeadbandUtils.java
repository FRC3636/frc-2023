
package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;

public class DeadbandUtils {
    public static double[] getXYWithDeadband(Joystick joystick, double deadband) {
        // Negative because joysticks are inverted
        double xValue = -joystick.getY() * (joystick.getZ() + 1) / 2;
        double yValue = -joystick.getX() * (joystick.getZ() + 1) / 2;

        // if one of the joystick axes are out of the deadzone, ignore it for both
        boolean force = Math.abs(xValue) > deadband || Math.abs(yValue) > deadband;

        xValue = applyDeadband(xValue, deadband, 1.0, force);
        yValue = applyDeadband(yValue, deadband, 1.0, force);

        return new double[] { xValue, yValue };
    }

    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.
    /**
     * Returns 0.0 if the given value is within the specified range around zero. The
     * remaining range
     * between the deadband and the maximum magnitude is scaled from 0.0 to the
     * maximum magnitude.
     *
     * @param value        Value to clip.
     * @param deadband     Range around zero.
     * @param maxMagnitude The maximum magnitude of the input. Can be infinite.
     * @param force        Skip the deadband check and only scale the value
     * @return The value after the deadband is applied.
     */
    private static double applyDeadband(double value, double deadband, double maxMagnitude, boolean force) {
        if (Math.abs(value) > deadband || force) {
            if (maxMagnitude / deadband > 1.0e12) {
                // If max magnitude is sufficiently large, the implementation encounters
                // roundoff error. Implementing the limiting behavior directly avoids
                // the problem.
                return value > 0.0 ? value - deadband : value + deadband;
            }
            if (value > 0.0) {
                // Map deadband to 0 and map max to max.
                //
                // y - y₁ = m(x - x₁)
                // y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                // y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                //
                // (x₁, y₁) = (deadband, 0) and (x₂, y₂) = (max, max).
                // x₁ = deadband
                // y₁ = 0
                // x₂ = max
                // y₂ = max
                //
                // y = (max - 0)/(max - deadband) (x - deadband) + 0
                // y = max/(max - deadband) (x - deadband)
                // y = max (x - deadband)/(max - deadband)
                return maxMagnitude * (value - deadband) / (maxMagnitude - deadband);
            } else {
                // Map -deadband to 0 and map -max to -max.
                //
                // y - y₁ = m(x - x₁)
                // y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                // y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                //
                // (x₁, y₁) = (-deadband, 0) and (x₂, y₂) = (-max, -max).
                // x₁ = -deadband
                // y₁ = 0
                // x₂ = -max
                // y₂ = -max
                //
                // y = (-max - 0)/(-max + deadband) (x + deadband) + 0
                // y = max/(max - deadband) (x + deadband)
                // y = max (x + deadband)/(max - deadband)
                return maxMagnitude * (value + deadband) / (maxMagnitude - deadband);
            }
        } else {
            return 0.0;
        }
    }
}
