package frc.robot.subsystems.arm

import com.revrobotics.AbsoluteEncoder
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import frc.robot.RobotContainer

open class Shoulder(protected val arm: Arm) {
    private val motor1 = CANSparkMax(Constants.Shoulder.MOTOR_1_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless)
    private val motor2 = CANSparkMax(Constants.Shoulder.MOTOR_2_ID,
            CANSparkMaxLowLevel.MotorType.kBrushless)
    private val encoder: AbsoluteEncoder = motor1.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    protected var feedforwardController = ArmFeedforward(Constants.Shoulder.KS, Constants.Shoulder.KG,
            Constants.Shoulder.KV, Constants.Shoulder.KA)
    protected val pidController = PIDController(Constants.Shoulder.KP, Constants.Shoulder.KI,
            Constants.Shoulder.KD)

    open val angle: Rotation2d
        get() = Rotation2d.fromRadians(
                if (encoder.position > Constants.Shoulder.MAX_ANGLE.radians
                        && motor1.encoder.position < Constants.Shoulder.TOLERANCE_ANGLE.radians
                ) encoder.position - 2 * Math.PI * Constants.Shoulder.GEAR_RATIO
                else encoder.position
        )
    open val velocity: Rotation2d
        get() = Rotation2d.fromRadians(encoder.velocity)

    init {
        motor1.restoreFactoryDefaults()
        motor2.restoreFactoryDefaults()
        motor1.idleMode = CANSparkMax.IdleMode.kBrake
        motor2.idleMode = CANSparkMax.IdleMode.kBrake
        motor1.setSmartCurrentLimit(40)
        motor2.setSmartCurrentLimit(40)
        motor2.follow(motor1, true)
        motor1.encoder.positionConversionFactor = Units.rotationsToRadians(1.0) / 151.2
        encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * Constants.Shoulder.GEAR_RATIO
        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * Constants.Shoulder.GEAR_RATIO / 60.0
        RobotContainer.armTab.add("Shoulder PID", pidController).withWidget(BuiltInWidgets.kPIDController)
        motor1.inverted = true
        encoder.inverted = true
        pidController.setTolerance(Units.degreesToRadians(1.0))
        motor1.encoder.position = angle.radians
    }

    open fun periodic() {
        SmartDashboard.putNumber("Shoulder Angle", angle.radians)
        SmartDashboard.putNumber("Shoulder Velocity", velocity.radians)
    }

    /// Run the shoulder at the given setpoints using the feedforward and feedback controllers.
    /// @param position The position setpoint. Measured in radians from the vertical.
    /// @param velocity The velocity setpoint. Measured in radians per second.
    /// @param acceleration The acceleration setpoint. Measured in radians per second squared.
    open fun runWithSetpoint(position: Rotation2d, velocity: Rotation2d, acceleration: Rotation2d) {
        var velocity = Rotation2d.fromRadians(velocity.radians +
                pidController.calculate(angle.radians,
                        position.radians.coerceAtLeast(Arm.State.Stowed.shoulderAngle.radians)
                )
        )
        val voltage = feedforwardController.calculate(
                angle.radians - Math.PI / 2,
                velocity.radians,
                acceleration.radians
        )
        motor1.setVoltage(voltage)
    }

    companion object {
        fun signedModularDistance(a: Double, b: Double, modulus: Double): Double {
            var a = a
            var b = b
            a = (a % modulus + a) % modulus
            b = (b % modulus + b) % modulus
            val posDist: Double
            val negDist: Double
            if (b >= a) {
                posDist = b - a
                negDist = a + (modulus - b)
            } else {
                posDist = b + (modulus - a)
                negDist = a - b
            }
            return if (posDist > negDist) {
                posDist
            } else {
                -negDist
            }
        }
    }
}