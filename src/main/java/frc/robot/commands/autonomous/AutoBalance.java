package frc.robot.commands.autonomous;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.*;

public class AutoBalance implements Command{
    
    int NumOscillation = 0;
    Rotation2d lastInclineAngle;
    double lastDistance = 0;
    private final Drivetrain drivetrain;
    //private final PIDController pidController = new PIDController(0, 0, 0);

    public AutoBalance(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        //RobotContainer.autoTab.add("AutoBalance", pidController).withWidget(BuiltInWidgets.kPIDController);
    }

    @Override
    public void initialize() {
        NumOscillation = 0;

        Rotation3d rot = drivetrain.getRotation3d();
        Translation3d normal = new Translation3d(0, 0, 1).rotateBy(rot);
        Translation2d inclineDirection = normal.toTranslation2d().rotateBy(drivetrain.getRotation().unaryMinus());
        lastDistance = inclineDirection.getDistance(new Translation2d(0, 0));
        lastInclineAngle = inclineDirection.getAngle();
    }

    @Override
    public void execute() {
        Rotation3d rot = drivetrain.getRotation3d();

        Translation3d normal = new Translation3d(0, 0, 1).rotateBy(rot);
        Translation2d inclineDirection = normal.toTranslation2d().rotateBy(drivetrain.getRotation().unaryMinus());
        
        double distance = inclineDirection.getDistance(new Translation2d(0, 0));
        if (distance < Math.sin(Constants.DriveConstants.CHARGE_TOLERANCE)/* || (distance - lastDistance) < -0.01*/) {
            drivetrain.setX();
            //lastDistance = distance;
            return;
        }
        //lastDistance = distance;

        Rotation2d inclineAngle = inclineDirection.getAngle();

        if (inclineAngle.minus(lastInclineAngle).getRadians() > Math.PI/2) {
            NumOscillation ++;
        }
        lastInclineAngle = inclineAngle;

        SmartDashboard.putNumber("InclineAngle: ", inclineAngle.getDegrees());

        //double driveSpeed = pidController.calculate(inclineDirection.getDistance(new Translation2d(0, 0)), 0);
        double driveSpeed = MathUtil.clamp(distance/Math.sin(Math.toRadians(15)), -1, 1 );
        if (distance < Math.sin(Constants.DriveConstants.CHARGE_TIPPING_ANGLE)) {
             driveSpeed *= Constants.DriveConstants.CHARGE_REDUCED_SPEED;
        } else {
            driveSpeed *= Constants.DriveConstants.CHARGE_MAX_SPEED;
        }
        driveSpeed /= NumOscillation + 1; // dampen the robot's oscillations by slowing down after oscillating
        Translation2d driveVelocity = new Translation2d(driveSpeed, inclineAngle);

        ChassisSpeeds speeds = new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), 0);

        drivetrain.drive(speeds);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
