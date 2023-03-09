package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants.AutoConstants
import frc.robot.poseestimation.PoseEstimation
import frc.robot.subsystems.drivetrain.Drivetrain

//Uses PID to move the robot to the specified Pose2d
class PIDDriveToPoint(private val drivetrain: Drivetrain, private val poseEstimation: PoseEstimation, private val target: Pose2d?) : CommandBase() {
    private val xController = PIDController(AutoConstants.P_TRANSLATION_POINT_CONTROLLER, 0.0, 0.0)
    private val yController = PIDController(AutoConstants.P_TRANSLATION_POINT_CONTROLLER, 0.0, 0.0)
    private val thetaController = PIDController(AutoConstants.P_THETA_POINT_CONTROLLER, 0.0, 0.0)

    init {
        xController.setTolerance(AutoConstants.TRANSLATION_TOLERANCE)
        yController.setTolerance(AutoConstants.TRANSLATION_TOLERANCE)
        thetaController.setTolerance(AutoConstants.THETA_TOLERANCE.radians)
        addRequirements(drivetrain)
    }

    override fun initialize() {
        xController.setpoint = target!!.x
        yController.setpoint = target.y
        thetaController.setpoint = target.rotation.radians
    }

    override fun execute() {
        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xController.calculate(poseEstimation.estimatedPose.x),
                        yController.calculate(poseEstimation.estimatedPose.y),
                        thetaController.calculate(poseEstimation.estimatedPose.rotation.radians),
                        poseEstimation.estimatedPose.rotation
                )
        )
    }

    override fun isFinished(): Boolean {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()
    }
}