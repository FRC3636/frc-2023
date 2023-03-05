package frc.robot.utils;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Auto {
    public static enum StartingPosition {
        Left,
        Middle,
        Right
    }

    public static enum ScoringType {
        Balance,
        Score
    }

    public static SendableChooser<StartingPosition> startingPosition = new SendableChooser<>();
    public static SendableChooser<ScoringType> scoringType = new SendableChooser<>();

    public static void init() {
        RobotContainer.autoTab.add("Starting Position", startingPosition)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(1, 1);

        RobotContainer.autoTab.add("Scoring Type", scoringType)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(1, 1);

        startingPosition.setDefaultOption("Left", StartingPosition.Left);
        startingPosition.addOption("Middle", StartingPosition.Middle);
        startingPosition.addOption("Right", StartingPosition.Right);

        scoringType.setDefaultOption("Balance", ScoringType.Balance);
        scoringType.addOption("Score", ScoringType.Score);
    }

    public static StartingPosition getStartingPosition() {
        return startingPosition.getSelected();
    }

    public static ScoringType getScoringType() {
        return scoringType.getSelected();
    }
    
}
