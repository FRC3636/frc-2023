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
            -Math.PI / 2,
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
        super.followShoulderWithVelocity(velocity.times(0.001).unaryMinus());
    }

    @Override
    public boolean isLimitSwitchPressed() {
        return false;
    }

    @Override
    public void runWithSetpoint(Rotation2d position, Rotation2d velocity) {
        if(velocity.getRadians() != 0) {
            Rotation2d minAngle = getWristAngleFromHeight(0.4);
            if(minAngle != null) {
                position = Rotation2d.fromRadians(Math.max(position.getRadians(), minAngle.getRadians()));
            }
        }

        velocity = Rotation2d.fromRadians(velocity.getRadians() + pidController.calculate(arm.getWristAngle().getRadians(), position.getRadians()));

        SmartDashboard.putNumber("Wrist Setpoint", position.getDegrees());

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
