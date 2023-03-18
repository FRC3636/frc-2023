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
import frc.robot.utils.DeadbandUtils;

public class DriveWithJoysticks implements Command {
    private final Drivetrain drivetrain;
    private final PoseEstimation poseEstimation;

    private final Joystick translation;
    private final Joystick rotation;

    private Rotation2d fieldOrientationZeroOffset = new Rotation2d();
    private Sensitivity sensitivity;

    public DriveWithJoysticks(Drivetrain drivetrain, PoseEstimation poseEstimation, Joystick translation,
            Joystick rotation) {
        this.drivetrain = drivetrain;
        this.poseEstimation = poseEstimation;

        this.translation = translation;
        this.rotation = rotation;
        drivetrain.resetEncoders();
        sensitivity = new Sensitivity(poseEstimation);
    }

    @Override
    public void execute() {
        double[] translationValues = DeadbandUtils.getXYWithDeadband(translation, DriveConstants.DEADZONE);
        double translationx = translationValues[0];
        double translationy = translationValues[1];
        double r = DeadbandUtils.getXYWithDeadband(rotation, DriveConstants.DEADZONE)[1];

        boolean withinDeadband = translationx == 0 && translationy == 0 && r == 0;

        double vx = translationx * DriveConstants.MAX_SPEED_METERS_PER_SECOND
                * sensitivity.getTranslationalSensitivity();
        double vy = translationy * DriveConstants.MAX_SPEED_METERS_PER_SECOND
                * sensitivity.getTranslationalSensitivity();
        double omega = r * DriveConstants.MAX_ANGULAR_SPEED * sensitivity.getRotationalSensitivity();

        ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds(vx, vy, omega);
        ChassisSpeeds robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelSpeeds,
                poseEstimation.getEstimatedPose().getRotation()
                        .minus(AllianceUtils.getFieldOrientationZero().plus(fieldOrientationZeroOffset)));

        if (!withinDeadband) {
            drivetrain.drive(robotRelSpeeds);
        } else {
            drivetrain.setX();
        }
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }

    public void resetFieldOrientation() {
        fieldOrientationZeroOffset = poseEstimation.getEstimatedPose().getRotation()
                .minus(AllianceUtils.getFieldOrientationZero());
    }
}
