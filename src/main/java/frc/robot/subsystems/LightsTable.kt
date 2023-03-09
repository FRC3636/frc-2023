package frc.robot.subsystems

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.NetworkTableValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.arm.Arm
import java.util.*

class LightsTable : SubsystemBase() {
    private val instance = NetworkTableInstance.getDefault()
    private val table = instance.getTable("Lights")
    private var current: String? = null
    private var requestMode = false
    var lightsPreset: String?
        get() = current
        set(presetId) {
            if (current != presetId) {
                current = presetId
                table.putValue("presetId", NetworkTableValue.makeString(presetId))
            }
        }

    override fun periodic() {
        lightsPreset = if (requestMode) {
            val targetPreset = Arm.State.getGamePiece().name.lowercase(Locale.getDefault())
            targetPreset
        } else {
            "rainbow"
        }
    }

    fun setEnabled(enabled: Boolean) {
        table.putValue("enabled", NetworkTableValue.makeBoolean(enabled))
    }

    fun setRequestMode(enabled: Boolean) {
        requestMode = enabled
    }
}