package frc.robot.commands.autonomous

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.Constants.DriveConstants
import frc.robot.subsystems.drivetrain.Drivetrain
import kotlin.math.sin

class AutoBalance(private val drivetrain: Drivetrain) : Command {
    var numOscillation = 0
    var lastInclineAngle: Rotation2d? = null
    var lastDistance = 0.0

    override fun initialize() {
        numOscillation = 0
        val rot = drivetrain.rotation3d
        val normal = Translation3d(0.0, 0.0, 1.0).rotateBy(rot)
        val inclineDirection = normal.toTranslation2d().rotateBy(drivetrain.rotation!!.unaryMinus())
        lastDistance = inclineDirection.getDistance(Translation2d(0.0, 0.0))
        lastInclineAngle = inclineDirection.angle
    }

    override fun execute() {
        val rot = drivetrain.rotation3d
        val normal = Translation3d(0.0, 0.0, 1.0).rotateBy(rot)
        val inclineDirection = normal.toTranslation2d().rotateBy(drivetrain.rotation!!.unaryMinus())
        val distance = inclineDirection.getDistance(Translation2d(0.0, 0.0))
        if (distance < sin(DriveConstants.CHARGE_STATION_TOLERANCE) /* || (distance - lastDistance) < -0.01*/) {
            drivetrain.setX()
            //lastDistance = distance;
            return
        }
        //lastDistance = distance;
        val inclineAngle = inclineDirection.angle
        if (inclineAngle.minus(lastInclineAngle).radians > Math.PI / 2) {
            numOscillation++
        }
        lastInclineAngle = inclineAngle
        SmartDashboard.putNumber("InclineAngle: ", inclineAngle.degrees)

        //double driveSpeed = pidController.calculate(inclineDirection.getDistance(new Translation2d(0, 0)), 0);
        var driveSpeed = MathUtil.clamp(0.65 * distance / Math.sin(Math.toRadians(15.0)), -1.0, 1.0)
        driveSpeed /= (numOscillation + 1).toDouble() // dampen the robot's oscilations by slowing down after oscilating
        val driveVelocity = Translation2d(driveSpeed, inclineAngle)
        val speeds = ChassisSpeeds(driveVelocity.x, driveVelocity.y, 0.0)
        drivetrain.drive(speeds)
    }

    override fun getRequirements(): Set<Subsystem> {
        return mutableSetOf<Subsystem>(drivetrain)
    }
}