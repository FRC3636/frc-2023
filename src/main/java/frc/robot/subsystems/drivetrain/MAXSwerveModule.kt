// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.drivetrain

import com.revrobotics.*
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.robot.Constants.ModuleConstants

class MAXSwerveModule(drivingCANId: Int, turningCANId: Int, chassisAngularOffset: Double) : SwerveModule {
    private val turningSparkMax: CANSparkMax
    private val drivingEncoder: RelativeEncoder
    val turningEncoder: AbsoluteEncoder // fixme
    private val drivingPIDController: SparkMaxPIDController
    private val turningPIDController: SparkMaxPIDController
    private val chassisAngularOffset: Double
    override var setState: SwerveModuleState = SwerveModuleState(0.0, Rotation2d())
        private set

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    init {
        val drivingSparkMax = CANSparkMax(drivingCANId, CANSparkMaxLowLevel.MotorType.kBrushless)
        turningSparkMax = CANSparkMax(turningCANId, CANSparkMaxLowLevel.MotorType.kBrushless)

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        drivingSparkMax.restoreFactoryDefaults()
        turningSparkMax.restoreFactoryDefaults()

        // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        drivingEncoder = drivingSparkMax.encoder
        turningEncoder = turningSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        drivingPIDController = drivingSparkMax.pidController
        turningPIDController = turningSparkMax.pidController
        drivingPIDController.setFeedbackDevice(drivingEncoder)
        turningPIDController.setFeedbackDevice(turningEncoder)

        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.
        drivingEncoder.positionConversionFactor = ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR
        drivingEncoder.velocityConversionFactor = ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        turningEncoder.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_FACTOR)
        turningEncoder.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR)

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        turningEncoder.setInverted(ModuleConstants.TURNING_ENCODER_INVERTED)

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        turningPIDController.positionPIDWrappingEnabled = true
        turningPIDController.positionPIDWrappingMinInput = ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT
        turningPIDController.positionPIDWrappingMaxInput = ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT

        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        drivingPIDController.p = ModuleConstants.DRIVING_P
        drivingPIDController.i = ModuleConstants.DRIVING_I
        drivingPIDController.d = ModuleConstants.DRIVING_D
        drivingPIDController.ff = ModuleConstants.DRIVING_FF
        drivingPIDController.setOutputRange(ModuleConstants.DRIVING_MIN_OUTPUT,
                ModuleConstants.DRIVING_MAX_OUTPUT)

        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        turningPIDController.p = ModuleConstants.TURNING_P
        turningPIDController.i = ModuleConstants.TURNING_I
        turningPIDController.d = ModuleConstants.TURNING_D
        turningPIDController.ff = ModuleConstants.TURNING_FF
        turningPIDController.setOutputRange(ModuleConstants.TURNING_MIN_OUTPUT,
                ModuleConstants.TURNING_MAX_OUTPUT)
        drivingSparkMax.idleMode = ModuleConstants.DRIVING_MOTOR_IDLE_MODE
        turningSparkMax.idleMode = ModuleConstants.TURNING_MOTOR_IDLE_MODE
        drivingSparkMax.setSmartCurrentLimit(ModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT)
        turningSparkMax.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT)

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        drivingSparkMax.burnFlash()
        turningSparkMax.burnFlash()
        this.chassisAngularOffset = chassisAngularOffset
        setState!!.angle = Rotation2d(turningEncoder.getPosition())
        drivingEncoder.position = 0.0
    }

    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    override val state: SwerveModuleState
        get() = SwerveModuleState(
                drivingEncoder.velocity,
                Rotation2d(turningEncoder.position - chassisAngularOffset)
        )

    override val position: SwerveModulePosition
        get() = SwerveModulePosition(
                    drivingEncoder.position,
                    Rotation2d(turningEncoder.position - chassisAngularOffset)
        )

    override fun setDesiredState(desiredState: SwerveModuleState) {
        // Apply chassis angular offset to the desired state.
        val correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset))

        // Optimize the reference state to avoid spinning further than 90 degrees.
        val optimizedDesiredState = SwerveModuleState.optimize(
                correctedDesiredState,
                Rotation2d(turningEncoder.position)
        )

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity)
        turningPIDController.setReference(optimizedDesiredState.angle.radians, CANSparkMax.ControlType.kPosition)
        setState = desiredState
    }

    override val swerveEncoderPosition: Double
        get() = turningEncoder.position

    /**
     * Zeroes all the SwerveModule encoders.
     */
    override fun resetEncoders() {
        drivingEncoder.position = 0.0
    }

    override fun update() {}
}