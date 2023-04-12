package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class NavXGyro implements Gyro{
    private final AHRS navX = new AHRS();
    private final GenericEntry gyroAdjustment = RobotContainer.swerveTab.add("Gyro Adjustment", DriveConstants.GYRO_ADJUSTMENT_RATE).getEntry();

    public NavXGyro() {
        navX.calibrate();
        reset();
    }

    @Override
    public boolean isConnected() {
        return navX.isConnected();
    }

    @Override
    public Rotation2d getAngle() {
        return navX.getRotation2d().rotateBy(getRate().times(gyroAdjustment.getDouble(Constants.DriveConstants.GYRO_ADJUSTMENT_RATE)));
    }

    @Override
    public Rotation3d getRotation3d() {
        return DriveConstants.GYRO_ROTATION.rotateBy(new Rotation3d(new Quaternion(
            navX.getQuaternionW(),
            navX.getQuaternionX(),
            navX.getQuaternionY(),
            navX.getQuaternionZ()
        ))).rotateBy(DriveConstants.GYRO_ROTATION.unaryMinus());
    }

    @Override
    public void setGyroRotation(Rotation2d rotation2d) {
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
        return Rotation2d.fromDegrees(-navX.getRate() * 60);
    }
}
