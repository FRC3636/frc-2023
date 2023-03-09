package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;

import java.util.function.Supplier;

public class PIDDriveToPoint extends CommandBase {

    private final Drivetrain drivetrain;
    private final PoseEstimation poseEstimation;
    private final Pose2d target;

    private final PIDController xController = new PIDController(Constants.AutoConstants.P_TRANSLATION_POINT_CONTROLLER, 0, 0);
    private final PIDController yController = new PIDController(Constants.AutoConstants.P_TRANSLATION_POINT_CONTROLLER, 0, 0);
    private final PIDController thetaController = new PIDController(Constants.AutoConstants.P_THETA_POINT_CONTROLLER, 0, 0);

    public PIDDriveToPoint(Drivetrain drivetrain, PoseEstimation poseEstimation, Pose2d target) {
        this.drivetrain = drivetrain;
        this.poseEstimation = poseEstimation;
        this.target = target;

        xController.setTolerance(Constants.AutoConstants.TRANSLATION_TOLERANCE);
        yController.setTolerance(Constants.AutoConstants.TRANSLATION_TOLERANCE);
        thetaController.setTolerance(Constants.AutoConstants.THETA_TOLERANCE.getRadians());

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xController.setSetpoint(target.getX());
        yController.setSetpoint(target.getY());
        thetaController.setSetpoint(target.getRotation().getRadians());
    }

    @Override
    public void execute() {
        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xController.calculate(poseEstimation.getEstimatedPose().getX()),
                        yController.calculate(poseEstimation.getEstimatedPose().getY()),
                        thetaController.calculate(poseEstimation.getEstimatedPose().getRotation().minus(target.getRotation()).getRadians(), 0),
                        poseEstimation.getEstimatedPose().getRotation()
                )
        );
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpfoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }
}
