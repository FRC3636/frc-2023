// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.utils.AllianceUtils;

import java.util.Map;

public class Sensitivity extends CommandBase {

    PoseEstimation poseEstimation;

    private final double TELLER_ZONE_TO_ALLIANCE = 13.15;
    private final double TELLER_ZONE_RANGE = 2.45;

    private GenericEntry knobSensitivityEnabled;

    private GenericEntry TellerSensitivityEntry;

    private GenericEntry TranslationalSensitivityEntry; 

    private GenericEntry RotationalSensitivityEntry;

    private GenericEntry TellerSensitivityDecreaseRate;

    public Sensitivity(PoseEstimation poseEstimation) {
        this.poseEstimation = poseEstimation;

        knobSensitivityEnabled = RobotContainer.driveSettingsTab
        .add("Knob Sensitivity Enabled", false)
        .withWidget(BuiltInWidgets.kBooleanBox).getEntry();

        TellerSensitivityEntry = RobotContainer.driveSettingsTab
        .add("Teller Sensitivity", getTellerSensitivity())
        .withWidget(BuiltInWidgets.kTextView).getEntry();


        TranslationalSensitivityEntry = RobotContainer.driveSettingsTab
        .add("Translational Sensitivity", 1)
        .withWidget(BuiltInWidgets.kTextView).getEntry();

        RotationalSensitivityEntry = RobotContainer.driveSettingsTab
        .add("Rotational Sensitivity", 1)
        .withWidget(BuiltInWidgets.kTextView).getEntry();

        TellerSensitivityDecreaseRate  = RobotContainer.driveSettingsTab
        .add("Teller Sensitivity Decrease Rate", 2)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 5)).getEntry();

        RobotContainer.driveSettingsTab.addNumber("Turn Sensitivity", RobotContainer.joystickRight::getZ);
        RobotContainer.driveSettingsTab.addNumber("Drive Sensitivity", RobotContainer.joystickLeft::getZ);
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
        if (inTellerZone() && RobotContainer.joystickLeft.getTrigger()) {
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
