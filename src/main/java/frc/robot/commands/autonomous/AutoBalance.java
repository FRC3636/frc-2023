package frc.robot.commands.autonomous;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.*;

public class AutoBalance implements Command {

    private int oscillationCount = 0;

    private Translation2d lastAscent;
    private Translation2d lastAscentWithDeadZone;

    private final Drivetrain drivetrain;
    //private final PIDController pidController = new PIDController(0, 0, 0);

    public AutoBalance(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        //RobotContainer.autoTab.add("AutoBalance", pidController).withWidget(BuiltInWidgets.kPIDController);
    }

    @Override
    public void initialize() {
        oscillationCount = 0;
        lastAscent = getAscentChassisRelative();
        lastAscentWithDeadZone = lastAscent;
    }

    @Override
    public void execute() {
        Translation2d ascent = getAscentChassisRelative();
        Rotation2d incline = new Rotation2d(Math.asin(ascent.getNorm()));
        Rotation2d deltaIncline = incline.minus(new Rotation2d(Math.asin(lastAscent.getNorm())));
        lastAscent = ascent;
        SmartDashboard.putNumber("Incline", incline.getDegrees());
        SmartDashboard.putNumber("Delta Incline", deltaIncline.getDegrees());

        if (incline.getRadians() <= Constants.DriveConstants.CHARGE_TOLERANCE.getRadians() ||
        ( incline.getRadians() < Rotation2d.fromDegrees(15).getRadians() 
        && deltaIncline.getRadians() < Constants.DriveConstants.CHARGE_ANGULAR_VELOCITY_TOLERANCE.getRadians())) {
            
            drivetrain.setX();
            return;
        }

        // for some reason there's no Translation2d::dot method
        if (oscillationCount < 4 && ascent.getX() * lastAscentWithDeadZone.getX() + ascent.getY() * lastAscentWithDeadZone.getY() < 0) {
            oscillationCount++;
            lastAscentWithDeadZone = ascent;
        }
        if (incline.getRadians() <= Constants.DriveConstants.CHARGE_TOLERANCE.getRadians()) {
            lastAscentWithDeadZone = ascent;
        }

        double driveSpeed = calculateDriveSpeed(incline, oscillationCount);
        Translation2d driveVelocity = ascent.div(ascent.getNorm()).times(driveSpeed);

        drivetrain.drive(new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), 0));
    }
    
    private Translation2d getAscentChassisRelative() {
        Rotation3d rot = drivetrain.getRotation3d();
        Translation3d normal = new Translation3d(0, 0, 1).rotateBy(rot);

        return normal.toTranslation2d().rotateBy(rot.toRotation2d().unaryMinus());
    }

    private static double calculateDriveSpeed(Rotation2d incline, int oscillationCount) {
        double driveSpeed;

        if (incline.getRadians() < Constants.DriveConstants.CHARGE_TIPPING_ANGLE.getRadians()) {
            driveSpeed = Constants.DriveConstants.CHARGE_REDUCED_SPEED;
        } else {
            driveSpeed = Constants.DriveConstants.CHARGE_MAX_SPEED * MathUtil.clamp(incline.getRadians() / Rotation2d.fromDegrees(15).getRadians(), -1, 1);;
        }

        driveSpeed /= Constants.DriveConstants.CHARGE_OSCILLATION_COEFFICIENT * (oscillationCount + 1); // dampen the robot's oscillations by slowing down after oscillating

        return driveSpeed;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drivetrain);
    }
}
