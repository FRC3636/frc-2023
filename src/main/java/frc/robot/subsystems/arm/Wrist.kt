package frc.robot.subsystems.arm

import com.revrobotics.AbsoluteEncoder
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import frc.robot.RobotContainer
import kotlin.math.asin
import kotlin.math.cos

open class Wrist(protected val arm: Arm) {
    private val motor = CANSparkMax(Constants.Wrist.ID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val limitSwitch = DigitalInput(Constants.Wrist.LIMIT_SWITCH)
    private val encoder: AbsoluteEncoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    @JvmField
    protected val pidController = PIDController(Constants.Wrist.KP, Constants.Wrist.KI, Constants.Wrist.KD)
    @JvmField
    protected val feedforward = ArmFeedforward(Constants.Wrist.KS, Constants.Wrist.KG, Constants.Wrist.KV, Constants.Wrist.KA)

    init {
        motor.restoreFactoryDefaults()
        motor.idleMode = CANSparkMax.IdleMode.kBrake
        encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * Constants.Wrist.GEAR_RATIO
        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * Constants.Wrist.GEAR_RATIO / 60.0
        motor.setSmartCurrentLimit(40)
        motor.inverted = false
        RobotContainer.armTab.add("Wrist PID", pidController).withWidget(BuiltInWidgets.kPIDController)
    }

    open val angle: Rotation2d?
        get() = Rotation2d.fromRadians(encoder.position)

    fun followShoulder() {
        followShoulderWithVelocity(arm.shoulderVelocity)
    }

    open fun followShoulderWithVelocity(velocity: Rotation2d) {
        var velocity = velocity
        if (Arm.State.target == Arm.State.Stowed && Arm.State.rollerSpeed == 0.0) {
            velocity = Rotation2d.fromRadians(velocity.radians + 1)
        }
        runWithSetpoint(setPosition, velocity)
    }

    fun runWithVelocity(velocity: Rotation2d) {
        runWithSetpoint(Rotation2d.fromRadians(encoder.position), velocity)
    }

    open fun runWithSetpoint(position: Rotation2d, velocity: Rotation2d) {
        var velocity = velocity
        velocity = Rotation2d.fromRadians(velocity.radians + pidController.calculate(arm.wristAngle.radians, position.radians))
        SmartDashboard.putNumber("Wrist Setpoint", position.radians)
        if (isLimitSwitchPressed && velocity.radians >= 0) {
            motor.set(0.0)
            return
        }
        motor.setVoltage(feedforward.calculate(arm.wristAngle.radians, velocity.radians))
    }

    fun getMinAngle(height: Double): Double {
        val intakeLength = Constants.Wrist.JOINT_TO_CORNER_DISTANCE
        val intakeAngleOffset = Constants.Wrist.HORIZONTAL_TO_CORNER_ANGLE
        val clearance = Constants.Wrist.CLEARANCE_HEIGHT
        //        System.out.println("Math vs Real angle diff(degrees)=" + ((angle-motor.getEncoder().getPosition()))*(360/2/Math.PI));
        return -asin((height - clearance) / intakeLength) + intakeAngleOffset
    }

    val setPosition: Rotation2d
        get() = if (arm.shoulderAngle.radians < Constants.Wrist.MIN_SHOULDER_ANGLE.radians
                || Arm.State.target.shoulderAngle.radians < Constants.Wrist.MIN_SHOULDER_ANGLE.radians) {
            Rotation2d.fromRadians(0.0.coerceAtLeast(Arm.State.target.wristAngle.radians))
        } else Arm.State.target.wristAngle

    fun safeHeight(armAngle: Double): Double {
        val armHeight = Constants.Arm.PIVOT_HEIGHT
        val armLength = Constants.Arm.HUMERUS_LENGTH
        return armHeight - cos(armAngle) * armLength
    }

    open val isLimitSwitchPressed: Boolean
        get() = !limitSwitch.get()

    open fun periodic() {
        SmartDashboard.putBoolean("Wrist Limit Switch", limitSwitch.get())
        SmartDashboard.putNumber("Wrist Angle", Units.radiansToDegrees(encoder.position))
        SmartDashboard.putNumber("Wrist Set Point", Arm.State.target.wristAngle.degrees)
        SmartDashboard.putNumber("Wrist Relative", arm.wristAngle.degrees)
        SmartDashboard.putNumber("minSafeAngle", 360.0 / 2.0 / Math.PI * getMinAngle(safeHeight(arm.shoulderAngle.degrees)))
    }
}