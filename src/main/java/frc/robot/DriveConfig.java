package frc.robot;

import java.util.HashMap;
import java.util.Map;

public class DriveConfig {
    private static final Map<String, DriveConfig> PRESETS = new HashMap<>();
    public static final String DEFAULT_PRESET_NAME = "default";

    static {
        PRESETS.put(DEFAULT_PRESET_NAME, new DriveConfig(1, 1, DriveScheme.Arcade));
        PRESETS.put("tank_drive", new DriveConfig(1, 1, DriveScheme.Tank));
        PRESETS.put("arcade_single", new DriveConfig(1, 1, DriveScheme.ArcadeSingle));
        PRESETS.put("person_2", new DriveConfig(0.5, 0.5, DriveScheme.Arcade));
    }

    /**
     * Provides the DriveConfig with the specified name
     *
     * @see DriveConfig.DEFAULT_PRESET_NAME
     */
    public static DriveConfig getPreset(String name) {
        return PRESETS.get(name);
    }

    public static DriveConfig getCurrent() {
        DriveConfig current = PRESETS.getOrDefault(RobotContainer.drivePresetsChooser.getSelected(),
                PRESETS.get(DEFAULT_PRESET_NAME));
        RobotContainer.updateDriveSchemeWidget(current.getDriveScheme());
        return current;
    }

    private final double speedSensitivity;
    private final double turnSensitivity;
    private final DriveScheme driveScheme;

    public DriveConfig(double speedSensitivity, double turnSensitivity, DriveScheme driveScheme) {
        this.speedSensitivity = speedSensitivity;
        this.turnSensitivity = turnSensitivity;
        this.driveScheme = driveScheme;
    }

    public double getSpeedSensitivity() {
        return speedSensitivity;
    }

    public double getTurnSensitivity() {
        return turnSensitivity;
    }

    public DriveScheme getDriveScheme() {
        return driveScheme;
    }

    public enum DriveScheme {
        Arcade,
        ArcadeSingle,
        Tank,
    }
}
