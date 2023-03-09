package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface Gyro {
    boolean isConnected();

    Rotation2d getAngle();

    Rotation3d getRotation3d();

    void setGyroRotation(Rotation2d rotation);

    void update();

    void reset();

    Rotation2d getRate();
}
