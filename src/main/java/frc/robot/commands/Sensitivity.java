// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.poseestimation.PoseEstimation;
import frc.robot.RobotContainer;
import frc.robot.utils.AllianceUtils;

public class Sensitivity extends CommandBase {

    PoseEstimation poseEstimation;

    private final double TELLER_ZONE_TO_ALLIANCE = 13.1;
    private final double TELLER_ZONE_RANGE = 2.45;

    private GenericEntry knobSensitivityEnabled = RobotContainer.driveSettingsTab
            .add("Knob Sensitivity Enabled", true)
            .withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    private GenericEntry TellerSensitivityEntry = RobotContainer.driveSettingsTab
            .add("Teller Sensitivity", getTellerSensitivity())
            .withWidget(BuiltInWidgets.kTextView).getEntry();

    private GenericEntry TranslationalSensitivityEntry = RobotContainer.driveSettingsTab
            .add("Translational Sensitivity", 1)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

    private GenericEntry RotationalSensitivityEntry = RobotContainer.driveSettingsTab
            .add("Rotational Sensitivity", 1)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

    private GenericEntry TellerSensitivityDecreaseRate = RobotContainer.driveSettingsTab
            .add("Teller Sensitivity Decrease Rate", 2)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 5)).getEntry();

    public Sensitivity(PoseEstimation poseEstimation) {
        this.poseEstimation = poseEstimation;
    }

    @Override
    public void execute() {
        TellerSensitivityEntry.setDouble(getTellerSensitivity());

        if(knobSensitivityEnabled.getBoolean(false)){
            TranslationalSensitivityEntry.setDouble(((RobotContainer.joystickLeft.getZ() + 1) / 2));
            RotationalSensitivityEntry.setDouble(((RobotContainer.joystickRight.getZ() + 1) / 2));
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public double getTellerSensitivity() {
        if (inTellerZone() && RobotContainer.joystickRight.getTrigger()) {
            double distanceInTellerZone = Math.min((TELLER_ZONE_RANGE + TELLER_ZONE_TO_ALLIANCE),
                    AllianceUtils.getDistanceFromAlliance(poseEstimation.getEstimatedPose())) - TELLER_ZONE_TO_ALLIANCE;

            double tellerSensitivity = 1
                    - (0.5 / Math.pow(TELLER_ZONE_RANGE, TellerSensitivityDecreaseRate.getDouble(2))
                            * (Math.pow(distanceInTellerZone, TellerSensitivityDecreaseRate.getDouble(2))));
            return tellerSensitivity;
        }
        return 1;
    }

    public double getRotationalSensitivity() {
        return knobSensitivityEnabled.getBoolean(false)
                ? RotationalSensitivityEntry.getDouble(1) * getTellerSensitivity()
                : getTellerSensitivity();
    }

    public double getTranslationalSensitivity() {
        return knobSensitivityEnabled.getBoolean(false)
                ? TranslationalSensitivityEntry.getDouble(1) * getTellerSensitivity()
                : getTellerSensitivity();
    }

    public boolean inTellerZone() {
        if (AllianceUtils.getDistanceFromAlliance(poseEstimation.getEstimatedPose()) >= TELLER_ZONE_TO_ALLIANCE) {
            return true;
        }
        return false;
    }
}
