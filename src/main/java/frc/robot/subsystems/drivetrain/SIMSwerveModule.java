package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

public class SIMSwerveModule implements SwerveModule{

    private final double chassisAngularOffset;

    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), Constants.ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR, 0.025);
    private FlywheelSim turnSim = new FlywheelSim(DCMotor.getNEO(1), Constants.ModuleConstants.TURNING_ENCODER_POSITION_FACTOR, 0.00096955);

    private final PIDController drivingPIDController;
    private final PIDController turningPIDController;

    private double drivePosition = 0.0;
    private double turnAbsolutePosition = 0.0;

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SIMSwerveModule(double chassisAngularOffset) {
        this.chassisAngularOffset = chassisAngularOffset;

        drivingPIDController = new PIDController(
                Constants.ModuleConstants.DRIVING_P,
                Constants.ModuleConstants.DRIVING_I,
                Constants.ModuleConstants.DRIVING_D
        );
        turningPIDController = new PIDController(
                Constants.ModuleConstants.TURNING_P,
                Constants.ModuleConstants.TURNING_I,
                Constants.ModuleConstants.TURNING_D
        );

        turningPIDController.enableContinuousInput(0, Math.PI * 2);

        desiredState.angle = Rotation2d.fromRadians(turnAbsolutePosition);
        drivePosition = 0;
        turnAbsolutePosition = chassisAngularOffset;
    }

    @Override
    public void update() {
        driveSim.setInputVoltage(MathUtil.clamp(
                drivingPIDController.calculate(driveSim.getAngularVelocityRadPerSec()), 
                -12, 12));

        driveSim.update(Robot.kDefaultPeriod);
        
        turnSim.setInputVoltage(MathUtil.clamp(
                turningPIDController.calculate(turnAbsolutePosition),
                -12, 12));

        turnSim.update(Robot.kDefaultPeriod);

        drivePosition += driveSim.getAngularVelocityRadPerSec() * Robot.kDefaultPeriod;
        turnAbsolutePosition = (turnAbsolutePosition + driveSim.getAngularVelocityRadPerSec() * Robot.kDefaultPeriod) % (Math.PI * 2);
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                drivePosition,
                new Rotation2d(turnAbsolutePosition - chassisAngularOffset));
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(turnAbsolutePosition));

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        drivingPIDController.setSetpoint(optimizedDesiredState.speedMetersPerSecond);
        turningPIDController.setSetpoint(optimizedDesiredState.angle.getRadians());

        this.desiredState = desiredState;
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveSim.getAngularVelocityRadPerSec(),
                new Rotation2d(turnAbsolutePosition - chassisAngularOffset));
    }

    @Override
    public double getSwerveEncoderPosition() {
        return turnAbsolutePosition;
    }

    @Override
    public void resetEncoders() {
        drivePosition = 0;
    }

    @Override
    public SwerveModuleState getSetState() {
        return desiredState;
    }
}
