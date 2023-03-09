package frc.robot.subsystems.drivetrain

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.robot.Constants

class SIMSwerveModule(private val chassisAngularOffset: Double) : SwerveModule {
    private val driveSim = FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025)
    private val turnSim = FlywheelSim(DCMotor.getNEO(1), Constants.ModuleConstants.TURNING_ENCODER_POSITION_FACTOR, 0.0001)
    private val drivePIDController: PIDController = PIDController(
            0.9,
            Constants.ModuleConstants.DRIVING_I,
            Constants.ModuleConstants.DRIVING_D
    )
    private val driveFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(0.116970, 0.133240)
    private val turnPIDController: PIDController = PIDController(
            7.0,
            Constants.ModuleConstants.TURNING_I,
            Constants.ModuleConstants.TURNING_D
    )
    private var drivePosition = 0.0
    override var swerveEncoderPosition = chassisAngularOffset
        private set
    private var desiredState = SwerveModuleState(0.0, Rotation2d.fromRadians(swerveEncoderPosition))
    override var setState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d())
        private set

    init {
        turnPIDController.enableContinuousInput(0.0, Math.PI * 2)
    }

    override fun update() {
        driveSim.setInputVoltage(MathUtil.clamp(driveFeedForward.calculate(desiredState.speedMetersPerSecond) +
                drivePIDController.calculate(driveSim.angularVelocityRadPerSec),
                -12.0, 12.0))
        driveSim.update(TimedRobot.kDefaultPeriod)
        turnSim.setInputVoltage(MathUtil.clamp(
                turnPIDController.calculate(swerveEncoderPosition),
                -12.0, 12.0))
        turnSim.update(TimedRobot.kDefaultPeriod)
        drivePosition += driveSim.angularVelocityRadPerSec * TimedRobot.kDefaultPeriod
        swerveEncoderPosition = (swerveEncoderPosition + turnSim.angularVelocityRadPerSec * TimedRobot.kDefaultPeriod) % (Math.PI * 2)
    }

    override val position: SwerveModulePosition
        get() = SwerveModulePosition(
                drivePosition,
                Rotation2d(swerveEncoderPosition - chassisAngularOffset))

    override fun setDesiredState(desiredState: SwerveModuleState) {
        // Apply chassis angular offset to the desired state.
        val correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset))

        // Optimize the reference state to avoid spinning further than 90 degrees.
        val optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                Rotation2d(swerveEncoderPosition))

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        drivePIDController.setpoint = optimizedDesiredState.speedMetersPerSecond
        turnPIDController.setpoint = optimizedDesiredState.angle.radians
        this.desiredState = optimizedDesiredState
        setState = desiredState
    }

    override val state: SwerveModuleState
        get() = SwerveModuleState(driveSim.angularVelocityRadPerSec,
                Rotation2d(swerveEncoderPosition - chassisAngularOffset))

    override fun resetEncoders() {
        drivePosition = 0.0
    }
}