package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;

import java.util.Objects;

public class LightsTable extends SubsystemBase {
    private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private final NetworkTable table = instance.getTable("Lights");

    private String current = null;
    private boolean requestMode = false;

    public LightsTable() {
    }

    public String getLightsPreset() {
        return current;
    }

    @Override
    public void periodic() {
        if(requestMode) {
            String targetPreset = Arm.State.getGamePiece().name().toLowerCase();
            this.setArmLights(targetPreset);
        } else {
            this.setArmLights("none");
        }
    }

    public void setArmLights(String presetId) {
        if(!Objects.equals(current, presetId)) {
            current = presetId;
            table.putValue("arm_section", NetworkTableValue.makeString(presetId));
        }
    }

    public void setEnabled(boolean enabled) {
        table.putValue("enabled", NetworkTableValue.makeBoolean(enabled));
    }

    public void setRequestMode(boolean enabled) {
        this.requestMode = enabled;
    }
}
