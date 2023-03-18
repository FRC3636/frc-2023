
package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;

public class DeadbandUtils {
    public static double[] getXYWithDeadband(Joystick joystick, double deadband) {
        // Negative because joysticks are inverted
        double xValue = -joystick.getY() * (joystick.getZ() + 1) / 2;
        double yValue = -joystick.getX() * (joystick.getZ() + 1) / 2;


        // only apply deadzone if both axes are inside
        if (Math.abs(xValue) < deadband && Math.abs(yValue) < deadband) {
            return new double[] { 0, 0 };
        }

        return new double[] { xValue, yValue };
    }
}
