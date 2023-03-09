package frc.robot.subsystems.arm

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.robot.Constants
import frc.robot.RobotContainer

class SIMShoulder(arm: Arm) : Shoulder(arm) {
    override val angle: Rotation2d
        get() = Rotation2d.fromRadians(shoulderSim.angleRads + Math.PI / 2)

    override val velocity: Rotation2d
        get() = Rotation2d.fromRadians(shoulderSim.velocityRadPerSec)

    private val shoulderSim: SingleJointedArmSim = SingleJointedArmSim(
            DCMotor.getNEO(2),
            115.2,
            0.663,
            Constants.Arm.HUMERUS_LENGTH,
            Constants.Shoulder.STOWED_ANGLE.radians - Math.PI / 2,
            Constants.Shoulder.HIGH_CONE_ANGLE.radians - Math.PI / 2,
            5.715,
            true
    )

    override fun runWithSetpoint(position: Rotation2d, velocity: Rotation2d, acceleration: Rotation2d) {
        val velocity = Rotation2d.fromRadians(velocity.radians +
                pidController.calculate(angle.radians,
                        position.radians + RobotContainer.joystickRight.z / 4
                )
        )
        val voltage = feedforwardController.calculate(
                angle.radians - Math.PI / 2,
                velocity.radians,
                acceleration.radians
        )
        shoulderSim.setInputVoltage(voltage)
    }

    override fun periodic() {
        super.periodic()
        shoulderSim.update(TimedRobot.kDefaultPeriod)
    }
}