package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
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
                    alliancePose.getRotation().unaryMinus()
                );
            default:
                return null;
        }
    }

    public static boolean isBlue() {
        return DriverStation.getAlliance().equals(Alliance.Blue);
    }
}
