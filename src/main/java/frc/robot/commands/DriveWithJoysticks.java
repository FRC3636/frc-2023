package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;

public class DriveWithJoysticks implements Command {
    private final Drivetrain drivetrain;
    private final PoseEstimation poseEstimation;

    private final Joystick translation;
    private final Joystick rotation;

    private Rotation2d fieldOrientationZero = Rotation2d.fromRadians( 
        AllianceUtils.isBlue() ? 0 : Math.PI
     );

    public DriveWithJoysticks(Drivetrain drivetrain, PoseEstimation poseEstimation, Joystick translation, Joystick rotation) {
        this.drivetrain = drivetrain;
        this.poseEstimation = poseEstimation;

        this.translation = translation;
        this.rotation = rotation;
    }

    @Override
    public void initialize() {
        fieldOrientationZero = Rotation2d.fromRadians(
                AllianceUtils.isBlue() ? 0 : Math.PI
        );
        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        double tx = MathUtil.applyDeadband(-translation.getY() * (translation.getZ() + 1)/2, 0.15);
        double ty = MathUtil.applyDeadband(-translation.getX() * (translation.getZ() + 1)/2, 0.15);
        double r = MathUtil.applyDeadband(-rotation.getX() * (rotation.getZ() + 1)/2, 0.15);

        double vx = tx * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
        double vy = ty * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
        double omega = r * DriveConstants.MAX_ANGULAR_SPEED;

        ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds(vx, vy, omega);
        ChassisSpeeds robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelSpeeds, poseEstimation.getEstimatedPose().getRotation().minus(fieldOrientationZero));

        drivetrain.drive(robotRelSpeeds);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }

    public void resetFieldOrientation() {
        fieldOrientationZero = poseEstimation.getEstimatedPose().getRotation();
    }
}
