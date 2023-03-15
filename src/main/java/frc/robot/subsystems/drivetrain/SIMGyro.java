package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

import java.util.Arrays;

public class SIMGyro implements Gyro {
    private final SwerveModules swerveModules;

    public SIMGyro(SwerveModules swerveModules) {
        this.swerveModules = swerveModules;
    }

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
    public Rotation3d getRotation3d() {
        return new Rotation3d();
    }

    @Override
    public void setGyroRotation(Rotation2d rotation) {
        this.rotation = rotation;
    }

    @Override
    public void update() {
        SwerveModuleState[] moduleStates = swerveModules.getStates().asArray();
        Translation2d velocity = Arrays.stream(moduleStates).map((state) -> new Translation2d(state.speedMetersPerSecond, state.angle)).reduce(Translation2d::plus).orElseThrow().div(moduleStates.length);

        SwerveModuleState referenceModule = swerveModules.getStates().frontLeft;
        Translation2d referenceModulePosition = new Translation2d(DriveConstants.TRACK_WIDTH, DriveConstants.WHEEL_BASE).div(2);

        Translation2d referenceModuleVelocity = new Translation2d(referenceModule.speedMetersPerSecond, referenceModule.angle);
        Translation2d referenceRotationalVelocityComponent = referenceModuleVelocity.minus(velocity);

        // FIXME: this is a really scuffed way of doing this
        double turningDirection = referenceRotationalVelocityComponent.getAngle().getRadians() > referenceModulePosition.getAngle().getRadians()  ? 1 : -1;

        turningRate = Rotation2d.fromRadians(referenceRotationalVelocityComponent.getNorm() * turningDirection / referenceModulePosition.getNorm());
        rotation = rotation.plus(turningRate.times(Robot.kDefaultPeriod));
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
