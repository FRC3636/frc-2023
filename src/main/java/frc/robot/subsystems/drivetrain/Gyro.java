package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyro {
    boolean isConnected();

    Rotation2d getAngle();

    void setGyroRotation(Rotation2d rotation);

    void update();

    void reset();

    Rotation2d getRate();
}
