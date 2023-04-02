package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class AllianceUtils {
    public static Pose2d allianceToField(Pose2d alliancePose) {
        switch (DriverStation.getAlliance()) {
            case Blue:
                return alliancePose;
            case Red:
                return new Pose2d(
                    new Translation2d(
                        FieldConstants.fieldLength - alliancePose.getX(),
                        alliancePose.getY()
                    ),
                    allianceToField(alliancePose.getRotation())
                );
            default:
                return null;
        }
    }

    public static Translation2d allianceToField(Translation2d alliancePose) {
        switch (DriverStation.getAlliance()) {
            case Blue:
                return alliancePose;
            case Red:
                return new Translation2d(
                        FieldConstants.fieldLength - alliancePose.getX(),
                        alliancePose.getY()
                );
            default:
                return null;
        }
    }

    public static Pose2d fieldToAlliance(Pose2d fieldPose) {
        switch (DriverStation.getAlliance()) {
            case Blue:
                return fieldPose;
            case Red:
                return new Pose2d(
                        new Translation2d(
                                FieldConstants.fieldLength - fieldPose.getX(),
                                fieldPose.getY()
                        ),
                        allianceToField(fieldPose.getRotation())
                );
            default:
                return null;
        }
    }

    public static boolean isBlue() {
        return DriverStation.getAlliance().equals(Alliance.Blue);
    }

    public static double getDistanceFromAlliance(Pose2d pose) {
        return isBlue()? pose.getX() : FieldConstants.fieldLength - pose.getX();
    }

    public static double allianceToField(double x) {
        return isBlue()? x : FieldConstants.fieldLength - x;
    }

    public static Rotation2d allianceToField(Rotation2d allianceRotation) {
        return isBlue() ? allianceRotation : Rotation2d.fromRadians(Math.PI).minus(allianceRotation);
    }

    public static Pose2d mirrorByAlliance(Pose2d alliancePose) {
        return isBlue()? alliancePose : new Pose2d(-alliancePose.getX(), alliancePose.getY(), Rotation2d.fromRotations(0.5).minus(alliancePose.getRotation()));
    }

    public static Translation2d mirrorByAlliance(Translation2d allianceTranslation) {
        return isBlue()? allianceTranslation : new Translation2d(-allianceTranslation.getX(), allianceTranslation.getY());
    }

    public static Rotation2d mirrorByAlliance(Rotation2d allianceRotation) {
        return isBlue()? allianceRotation :Rotation2d.fromRotations(0.5).minus(allianceRotation);
    }

    public static Rotation2d getFieldOrientationZero() {
        return Rotation2d.fromRadians(isBlue() ? 0 : Math.PI);
    }
}
