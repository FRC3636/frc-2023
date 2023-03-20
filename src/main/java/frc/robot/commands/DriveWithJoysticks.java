package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;
import frc.robot.utils.DeadbandUtils;

import java.sql.Timestamp;
import java.util.Date;

public class DriveWithJoysticks implements Command {
    private final Drivetrain drivetrain;
    private final PoseEstimation poseEstimation;

    private final Joystick translation;
    private final Joystick rotation;

    private Timestamp prevTimestamp;

    private int xModeCooldown = 0;

    private Rotation2d fieldOrientationZeroOffset = new Rotation2d();
    private Sensitivity sensitivity;

    private final Field2d leftJoystickGraph = new Field2d();
    private final Field2d rightJoystickGraph = new Field2d();

    public DriveWithJoysticks(Drivetrain drivetrain, PoseEstimation poseEstimation, Joystick translation,
            Joystick rotation) {
        this.drivetrain = drivetrain;
        this.poseEstimation = poseEstimation;

        this.translation = translation;
        this.rotation = rotation;
        drivetrain.resetEncoders();
        sensitivity = new Sensitivity(poseEstimation);

        RobotContainer.swerveTab.add("Left Joystick", leftJoystickGraph).withWidget(BuiltInWidgets.kField).withSize(3, 2);
        RobotContainer.swerveTab.add("Right Joystick", rightJoystickGraph).withWidget(BuiltInWidgets.kField).withSize(3, 2);
    }

    private static void updateJoystickGraph(Field2d graph, double[] joystickValues) {
        graph.setRobotPose(joystickValues[0] + FieldConstants.fieldLength / 2, joystickValues[1] + FieldConstants.fieldWidth / 2, new Rotation2d());
    }

    @Override
    public void execute() {
        if(xModeCooldown > 0){
            xModeCooldown -= prevTimestamp.compareTo(new Date());
        }
        
        double[] translationValues = DeadbandUtils.getXYWithDeadband(translation, DriveConstants.DEADZONE);
        double translationx = translationValues[0];
        double translationy = translationValues[1];
        updateJoystickGraph(leftJoystickGraph, translationValues);
        double[] rotationValues = DeadbandUtils.getXYWithDeadband(rotation, DriveConstants.DEADZONE);
        double r = rotationValues[1];
        updateJoystickGraph(rightJoystickGraph, rotationValues);

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

        if (withinDeadband && xModeCooldown ==0) {
            drivetrain.setX();
            xModeCooldown = 1000;
            prevTimestamp = new Timestamp(new Date().getTime());

        } else {
            drivetrain.drive(robotRelSpeeds);
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
