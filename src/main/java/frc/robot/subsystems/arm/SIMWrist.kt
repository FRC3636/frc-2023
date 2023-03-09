package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

public class SIMWrist extends Wrist{

    private final SingleJointedArmSim wristSim = new SingleJointedArmSim(
            DCMotor.getNEO(1),
            75.0,
            0.043,
            Constants.Wrist.HORIZONTAL_TO_CORNER_ANGLE,
            Constants.Wrist.CONE_ANGLE.getRadians(),
            Constants.Wrist.LIMIT_SWITCH_OFFSET.getRadians(),
            2.827,
            true
    );


    public SIMWrist(Arm arm) {
        super(arm);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(wristSim.getAngleRads() - arm.getShoulderAngle().getRadians());
    }

    @Override
    public void followShoulderWithVelocity(Rotation2d velocity) {
        super.followShoulderWithVelocity(new Rotation2d());
    }

    @Override
    public boolean isLimitSwitchPressed() {
        return wristSim.hasHitUpperLimit();
    }

    @Override
    public void runWithSetpoint(Rotation2d position, Rotation2d velocity) {
        velocity = Rotation2d.fromRadians(velocity.getRadians() + pidController.calculate(arm.getWristAngle().getRadians(), position.getRadians()));

        SmartDashboard.putNumber("Wrist Setpoint", position.getRadians());

        if (isLimitSwitchPressed() && velocity.getRadians() >= 0) {
            wristSim.setInputVoltage(0);
            return;
        }

        wristSim.setInputVoltage(feedforward.calculate(arm.getWristAngle().getRadians(), velocity.getRadians()));
    }

    @Override
    public void periodic() {
        super.periodic();
        wristSim.update(Robot.kDefaultPeriod);
    }
}
