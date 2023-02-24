package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

public class SIMGyro implements Gyro {
    Rotation2d rotation = new Rotation2d();
    Rotation2d turningRate = new Rotation2d();

    @Override
    public boolean isConnected() {
        return true;
    }

    @Override
    public Rotation2d getAngle() {
        return rotation;
    }

    @Override
    public void setGyroRotation(Rotation2d rotation) {
        this.rotation = rotation;
    }

    @Override
    public void update() {

    }

    @Override
    public void reset() {
        rotation = new Rotation2d();
    }

    @Override
    public Rotation2d getRate() {
        return turningRate;
    }
}
