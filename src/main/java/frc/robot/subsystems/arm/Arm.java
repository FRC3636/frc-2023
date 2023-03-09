package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmMoveCommand;
import frc.robot.utils.Node;

public class Arm extends SubsystemBase {

    private final Shoulder shoulder;
    private final Wrist wrist;
    private final Rollers rollers;

    private final Mechanism2d arm = new Mechanism2d(Units.inchesToMeters(28), Constants.Arm.PIVOT_HEIGHT + 0.5);
    private final MechanismRoot2d pivot = arm.getRoot("pivot", Units.inchesToMeters(22), Constants.Arm.PIVOT_HEIGHT);
    private final MechanismLigament2d humerus = pivot.append(new MechanismLigament2d("humerus", Constants.Arm.HUMERUS_LENGTH, 0));
    private final MechanismLigament2d manipulator = humerus.append(new MechanismLigament2d("manipulator", Constants.Wrist.JOINT_TO_CORNER_DISTANCE, 0));

    private final MechanismLigament2d setHumerus = pivot.append(new MechanismLigament2d("set humerus", Constants.Arm.HUMERUS_LENGTH, 0));
    private final MechanismLigament2d setManipulator = setHumerus.append(new MechanismLigament2d("set manipulator", Constants.Wrist.JOINT_TO_CORNER_DISTANCE, 0));

    public Arm() {
        shoulder = (RobotBase.isSimulation())? new SIMShoulder(this): new Shoulder(this);
        wrist = (RobotBase.isSimulation())? new SIMWrist(this): new Wrist(this);
        rollers = new Rollers();

        setHumerus.setColor(new Color8Bit(0, 255, 255));
        setManipulator.setColor(new Color8Bit(0, 100, 255));

        RobotContainer.armTab.addDoubleArray("Arm Position", this::getArm3dPose);
        RobotContainer.armTab.add("Arm", arm);
    }

    @Override
    public void periodic() {
        shoulder.periodic();
        wrist.periodic();
        rollers.periodic();

        humerus.setAngle(getShoulderAngle().minus(Rotation2d.fromDegrees(90)));
        manipulator.setAngle(wrist.getAngle().plus(Rotation2d.fromDegrees(90)));

        setHumerus.setAngle(State.getTarget().getShoulderAngle().minus(Rotation2d.fromDegrees(90)));
        setManipulator.setAngle(State.target.getWristAngle().plus(Rotation2d.fromDegrees(90)).minus(State.getTarget().getShoulderAngle()));
    }

    public double[] getArm3dPose() {
        Pose3d shoulderPose = new Pose3d(
                Constants.Arm.PIVOT_FORWARD_OFFSET,
                0,
                Constants.Arm.PIVOT_HEIGHT,
                new Rotation3d(0, -getShoulderAngle().getRadians(), 0));

        Translation2d relativeWristOrigin = Constants.Arm.RELATIVE_WRIST_POSE.rotateBy(getShoulderAngle());

        Pose3d wristPose = new Pose3d(
                Constants.Arm.PIVOT_FORWARD_OFFSET + relativeWristOrigin.getX(),
                0,
                Constants.Arm.PIVOT_HEIGHT + relativeWristOrigin.getY(),
                new Rotation3d(0, -getWristAngle().getRadians(), 0));

        return new double[] {
                shoulderPose.getX(),
                shoulderPose.getY(),
                shoulderPose.getZ(),
                shoulderPose.getRotation().getQuaternion().getW(),
                shoulderPose.getRotation().getQuaternion().getX(),
                shoulderPose.getRotation().getQuaternion().getY(),
                shoulderPose.getRotation().getQuaternion().getZ(),
                wristPose.getX(),
                wristPose.getY(),
                wristPose.getZ(),
                wristPose.getRotation().getQuaternion().getW(),
                wristPose.getRotation().getQuaternion().getX(),
                wristPose.getRotation().getQuaternion().getY(),
                wristPose.getRotation().getQuaternion().getZ(),
        };
    }

    public Rotation2d getShoulderAngle() {
        return shoulder.getAngle();
    }

    public Rotation2d getShoulderVelocity() {
        return shoulder.getVelocity();
    }

    public void runWithSetpoint(Rotation2d shoulderPosition, Rotation2d velocity) {
        shoulder.runWithSetpoint(shoulderPosition, velocity, new Rotation2d());
        wrist.followShoulderWithVelocity(velocity.times(-1));
    }

    public Rotation2d getWristAngle() {
        return getShoulderAngle().plus(wrist.getAngle());
    }

    public enum State {
        Teller(Constants.Shoulder.TELLER_CONE_ANGLE, Constants.Shoulder.TELLER_CUBE_ANGLE, Constants.Wrist.TELLER_CONE_ANGLE, Constants.Wrist.TELLER_CUBE_ANGLE),
        Slide(Constants.Shoulder.SLIDE_CONE_ANGLE, Constants.Shoulder.SLIDE_CUBE_ANGLE, Constants.Wrist.SLIDE_CONE_ANGLE, Constants.Wrist.SLIDE_CUBE_ANGLE),
        High(Constants.Shoulder.HIGH_CONE_ANGLE, Constants.Shoulder.HIGH_CUBE_ANGLE, Constants.Wrist.HIGH_CONE_ANGLE, Constants.Wrist.HIGH_CUBE_ANGLE),
        Mid(Constants.Shoulder.MID_CONE_ANGLE, Constants.Shoulder.MID_CUBE_ANGLE, Constants.Wrist.MID_CONE_ANGLE, Constants.Wrist.MID_CUBE_ANGLE),
        Low(Constants.Shoulder.INTAKE_CONE_ANGLE, Constants.Shoulder.LOW_CUBE_ANGLE, Constants.Wrist.INTAKE_CONE_ANGLE, Constants.Wrist.LOW_CUBE_ANGLE),
        Stowed(Constants.Shoulder.STOWED_ANGLE, Constants.Shoulder.STOWED_ANGLE, Constants.Wrist.STOWED_ANGLE, Constants.Wrist.CUBE_ANGLE);

        private final Rotation2d shoulderCubeAngle;
        private final Rotation2d shoulderConeAngle;
        private final Rotation2d wristConeAngle;
        private final Rotation2d wristCubeAngle;

        private Rotation2d shoulderConeOffset = new Rotation2d();
        private Rotation2d shoulderCubeOffset = new Rotation2d();
        private Rotation2d wristConeOffset = new Rotation2d();
        private Rotation2d wristCubeOffset = new Rotation2d();

        private static State target = State.Stowed;
        private static GamePiece gamePiece = GamePiece.Cube;
        private static Rollers.State rollerState = Rollers.State.Off;

        State(Rotation2d shoulderConeAngle, Rotation2d shoulderCubeAngle, Rotation2d wristConeAngle, Rotation2d wristCubeAngle) {
            this.shoulderCubeAngle = shoulderCubeAngle;
            this.shoulderConeAngle = shoulderConeAngle;
            this.wristConeAngle = wristConeAngle;
            this.wristCubeAngle = wristCubeAngle;
        }

        State(Rotation2d shoulderAngle) {
            this.shoulderCubeAngle = shoulderAngle;
            this.shoulderConeAngle = shoulderAngle;

            this.wristConeAngle = defaultWristAngle(GamePiece.Cone);
            this.wristCubeAngle = defaultWristAngle(GamePiece.Cube);
        }

        public static void moveShoulderOffset(Rotation2d difference) {
            switch(gamePiece) {
                case Cone:
                    State.target.shoulderConeOffset = Rotation2d.fromRadians(State.target.shoulderConeOffset.getRadians() + difference.getRadians());
                    break;
                case Cube:
                    State.target.shoulderCubeOffset = Rotation2d.fromRadians(State.target.shoulderCubeOffset.getRadians() + difference.getRadians());
            }
        }
        public static void moveWristOffset(Rotation2d difference) {
            switch(gamePiece) {
                case Cone:
                    State.target.wristConeOffset = Rotation2d.fromRadians(State.target.wristConeOffset.getRadians() + difference.getRadians());
                    break;
                case Cube:
                    State.target.wristCubeOffset = Rotation2d.fromRadians(State.target.wristCubeOffset.getRadians() + difference.getRadians());
            }
        }

        public static void resetOffset() {
            switch(gamePiece) {
                case Cone:
                    State.target.wristConeOffset = new Rotation2d();
                    State.target.shoulderConeOffset = new Rotation2d();
                    break;
                case Cube:
                    State.target.wristCubeOffset = new Rotation2d();
                    State.target.shoulderCubeOffset = new Rotation2d();
            }
        }

        public Rotation2d getShoulderAngle() {
            return (gamePiece == GamePiece.Cone) ? shoulderConeAngle.plus(shoulderConeOffset) : shoulderCubeAngle.plus(shoulderCubeOffset);
        }

        public Rotation2d getWristAngle() {
            if(this == State.Stowed && rollerState == Rollers.State.Off) {
                return Constants.Wrist.LIMIT_SWITCH_OFFSET;
            }
            return(gamePiece == GamePiece.Cone) ? wristConeAngle.plus(wristConeOffset) : wristCubeAngle.plus(wristCubeOffset);
        }

        public Rotation2d defaultWristAngle(GamePiece gamePiece) {
            return (gamePiece == GamePiece.Cone)? Constants.Wrist.CONE_ANGLE : Constants.Wrist.CUBE_ANGLE;
        }

        public static State getTarget() {
            return target;
        }

        public static void setTargetFromNode(Node node) {
            State.gamePiece = node.getNodeType();
            switch (node.getLevel()) {
                case Low:
                    State.target = Low;
                    break;
                case Mid:
                    State.target = Mid;
                    break;
                case High:
                    State.target = High;
            }
            new ArmMoveCommand(RobotContainer.arm).schedule();
        }

        public static void setTarget(State target) {
            State.target = target;
            new ArmMoveCommand(RobotContainer.arm).schedule();
        }

        public static GamePiece getGamePiece() {
            return gamePiece;
        }

        public static void setGamePiece(GamePiece gamePiece) {
            State.gamePiece = gamePiece;
            new ArmMoveCommand(RobotContainer.arm).schedule();
        }

        public static void setRollerState(Rollers.State state) {
            State.rollerState = state;
            new ArmMoveCommand(RobotContainer.arm).schedule();
        }

        public static double getRollerSpeed() {
            return gamePiece == GamePiece.Cone ? rollerState.coneSpeed : rollerState.cubeSpeed;
        }

        public enum GamePiece {
            Cone,
            Cube
        }

        public static double[] getPresets(State value) {
            return new double[] {
                    value.shoulderConeAngle.plus(value.shoulderConeOffset).getRadians(),
                    value.shoulderCubeAngle.plus(value.shoulderCubeOffset).getRadians(),
                    value.wristConeAngle.plus(value.wristConeOffset).getRadians(),
                    value.wristCubeAngle.plus(value.wristCubeOffset).getRadians(),
            };
        }
    }
}
