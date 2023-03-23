package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.GamePiece;

public class SIMShoulder extends Shoulder {

    private final SingleJointedArmSim shoulderSim = new SingleJointedArmSim(
            DCMotor.getNEO(2),
            115.2,
            0.663,
            Constants.Arm.HUMERUS_LENGTH,
            Arm.State.Stowed.getShoulderAngleFor(GamePiece.Cone).getRadians() - Math.PI / 2,
            Arm.State.High.getShoulderAngleFor(GamePiece.Cone).getRadians() - Math.PI / 2,
            5.715,
            true
    );

    public SIMShoulder() {}

    @Override
    public Rotation2d getAngle() {
        if(shoulderSim == null) {
            return new Rotation2d();
        }
        return Rotation2d.fromRadians(shoulderSim.getAngleRads() + Math.PI / 2);
    }

    @Override
    public Rotation2d getVelocity() {
        return Rotation2d.fromRadians(shoulderSim.getVelocityRadPerSec());
    }

    @Override
    public void runWithSetpoint(Rotation2d position, Rotation2d velocity, Rotation2d acceleration) {
        velocity = Rotation2d.fromRadians(velocity.getRadians() +
                        pidController.calculate(getAngle().getRadians(), position.getRadians())
        );

        double voltage = feedforwardController.calculate(
                getAngle().getRadians() - Math.PI / 2,
                velocity.getRadians(),
                acceleration.getRadians()
        );

        shoulderSim.setInputVoltage(voltage);
    }

    @Override
    public void periodic() {
        super.periodic();
        shoulderSim.update(Robot.kDefaultPeriod);
    }
}
