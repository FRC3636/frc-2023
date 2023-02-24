package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModules {
    public final SwerveModule frontLeft;
    public final SwerveModule frontRight;
    public final SwerveModule rearLeft;
    public final SwerveModule rearRight;

    private final SwerveModule[] modules;

    public SwerveModules(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule rearLeft, SwerveModule rearRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;

        this.modules = new SwerveModule[4];
        this.modules[Corner.FrontLeft.index] = frontLeft;
        this.modules[Corner.FrontRight.index] = frontRight;
        this.modules[Corner.RearLeft.index] = rearLeft;
        this.modules[Corner.RearRight.index] = rearRight;
    }

    public void update() {
        for (SwerveModule module : modules) {
            module.update();
        }
    }

    public SwerveModule[] asArray() {
        return this.modules;
    }

    public States getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i = 0; i < this.modules.length; i++) {
            states[i] = this.modules[i].getState();
        }

        return new States(states);
    }

    public Positions getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (int i = 0; i < this.modules.length; i++) {
            positions[i] = this.modules[i].getPosition();
        }

        return new Positions(positions);
    }

    public void resetEncoders() {
        for (SwerveModule module : this.modules) {
            module.resetEncoders();
        }
    }

    public void setDesiredStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < desiredStates.length; i++) {
            this.modules[i].setDesiredState(desiredStates[i]);
        }
    }

    public static class States {
        public final SwerveModuleState frontLeft;
        public final SwerveModuleState frontRight;
        public final SwerveModuleState rearLeft;
        public final SwerveModuleState rearRight;

        private final SwerveModuleState[] states;

        public States(SwerveModuleState[] states) {
            this.states = states;

            this.frontLeft = this.states[Corner.FrontLeft.index];
            this.frontRight = this.states[Corner.FrontRight.index];
            this.rearLeft = this.states[Corner.RearLeft.index];
            this.rearRight = this.states[Corner.RearRight.index];
        }

        public States(SwerveModuleState frontLeft, SwerveModuleState frontRight, SwerveModuleState rearLeft, SwerveModuleState rearRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.rearLeft = rearLeft;
            this.rearRight = rearRight;

            this.states = new SwerveModuleState[4];
            this.states[Corner.FrontLeft.index] = frontLeft;
            this.states[Corner.FrontRight.index] = frontRight;
            this.states[Corner.RearLeft.index] = rearLeft;
            this.states[Corner.RearRight.index] = rearRight;
        }

        public SwerveModuleState[] asArray() {
            return this.states;
        }
    }

    public static class Positions {
        public final SwerveModulePosition frontLeft;
        public final SwerveModulePosition frontRight;
        public final SwerveModulePosition rearLeft;
        public final SwerveModulePosition rearRight;

        private final SwerveModulePosition[] positions;

        public Positions(SwerveModulePosition[] positions) {
            this.positions = positions;

            this.frontLeft = this.positions[Corner.FrontLeft.index];
            this.frontRight = this.positions[Corner.FrontRight.index];
            this.rearLeft = this.positions[Corner.RearLeft.index];
            this.rearRight = this.positions[Corner.RearRight.index];
        }

        public SwerveModulePosition[] asArray() {
            return this.positions;
        }
    }

    public enum Corner {
        FrontLeft(0),
        FrontRight(1),
        RearLeft(2),
        RearRight(3);

        public final int index;

        Corner(int index) {
            this.index = index;
        }
    }
}
