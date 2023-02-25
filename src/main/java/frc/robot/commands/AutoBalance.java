package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance implements Command{

    Drivetrain drivetrain;
    private final PIDController pidController = new PIDController(0, 0,
    0);

    public AutoBalance(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        RobotContainer.autoTab.add("AutoBalance", pidController).withWidget(BuiltInWidgets.kPIDController);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Rotation3d rot = drivetrain.getRotation3d();

        Translation3d normal = new Translation3d(0, 0, 1).rotateBy(rot);
        Translation2d inclineDirection = normal.toTranslation2d().rotateBy(drivetrain.getRotation().unaryMinus());
        Rotation2d inclineAngle = inclineDirection.getAngle();

        SmartDashboard.putNumber("InclineAngle: ", inclineAngle.getDegrees());

        double driveSpeed = pidController.calculate(inclineDirection.getDistance(new Translation2d(0, 0)), 0);
        //0.4 * MathUtil.applyDeadband(inclineDirection.times(5).getDistance(new Translation2d(0, 0)), 0.3);
        Translation2d driveVelocity = new Translation2d(-driveSpeed, inclineAngle);

        ChassisSpeeds speeds = new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), 0);


        drivetrain.drive(speeds);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
