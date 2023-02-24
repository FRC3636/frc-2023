package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;

public class NavXGyro implements Gyro{
    private final AHRS navX = new AHRS();

    @Override
    public boolean isConnected() {
        return navX.isConnected();
    }

    @Override
    public Rotation2d getAngle() {
        return navX.getRotation2d();
    }

    @Override
    public void SetGyroRotation(Rotation2d rotation2d) {
        navX.reset();
        navX.setAngleAdjustment(-rotation2d.getDegrees()); // Probably non-functional
    }

    @Override
    public void update() {

    }

    @Override
    public void reset() {
        navX.reset();
    }

    @Override
    public Rotation2d getRate() {
        return Rotation2d.fromDegrees(navX.getRate());
    }
}
