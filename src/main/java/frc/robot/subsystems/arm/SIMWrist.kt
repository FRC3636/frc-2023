package frc.robot.subsystems.arm

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants

class SIMWrist(arm: Arm) : Wrist(arm) {
    private val wristSim = SingleJointedArmSim(
            DCMotor.getNEO(1),
            75.0,
            0.043,
            Constants.Wrist.HORIZONTAL_TO_CORNER_ANGLE,
            Constants.Wrist.CONE_ANGLE.radians,
            Constants.Wrist.LIMIT_SWITCH_OFFSET.radians,
            2.827,
            true
    )
    override val angle: Rotation2d
        get() = Rotation2d.fromRadians(wristSim.angleRads - arm.shoulderAngle.radians)

    override fun followShoulderWithVelocity(velocity: Rotation2d) {
        super.followShoulderWithVelocity(Rotation2d())
    }

    override val isLimitSwitchPressed: Boolean
        get() = wristSim.hasHitUpperLimit()

    override fun runWithSetpoint(position: Rotation2d, velocity: Rotation2d) {
        var velocity = velocity
        velocity = Rotation2d.fromRadians(velocity.radians + pidController.calculate(arm.wristAngle.radians, position.radians))
        SmartDashboard.putNumber("Wrist Setpoint", position.radians)
        if (isLimitSwitchPressed && velocity.radians >= 0) {
            wristSim.setInputVoltage(0.0)
            return
        }
        wristSim.setInputVoltage(feedforward.calculate(arm.wristAngle.radians, velocity.radians))
    }

    override fun periodic() {
        super.periodic()
        wristSim.update(TimedRobot.kDefaultPeriod)
    }
}