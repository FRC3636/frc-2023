package frc.robot.subsystems.arm

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import frc.robot.Constants

class Rollers {
    private val motor = CANSparkMax(Constants.Rollers.ID, CANSparkMaxLowLevel.MotorType.kBrushless)

    init {
        motor.idleMode = CANSparkMax.IdleMode.kBrake
        motor.restoreFactoryDefaults()
    }

    fun periodic() {
        motor.set(Arm.State.rollerSpeed)
    }

    enum class State(val coneSpeed: Double, val cubeSpeed: Double) {
        Intake(Constants.Rollers.INTAKE_CONE, Constants.Rollers.INTAKE_CUBE), Outtake(Constants.Rollers.OUTTAKE_CONE, Constants.Rollers.OUTTAKE_CUBE), Off(0.0, 0.0);
    }
}