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
import frc.robot.utils.GamePiece;
import frc.robot.utils.Node;

public class Arm extends SubsystemBase {

    private State target = State.Stowed;
    private GamePiece gamePiece = GamePiece.Cube;
    private Rollers.State rollerState = Rollers.State.Off;

    private final Shoulder shoulder;
    private final Wrist wrist;
    private final Rollers rollers;

    private final Mechanism2d arm = new Mechanism2d(Units.inchesToMeters(28), Constants.Arm.PIVOT_HEIGHT + 0.5);
    private final MechanismRoot2d pivot = arm.getRoot("pivot", Units.inchesToMeters(22), Constants.Arm.PIVOT_HEIGHT);
    private final MechanismLigament2d humerus = pivot
            .append(new MechanismLigament2d("humerus", Constants.Arm.HUMERUS_LENGTH, 0));
    private final MechanismLigament2d manipulator = humerus
            .append(new MechanismLigament2d("manipulator", Constants.Wrist.JOINT_TO_CORNER_DISTANCE, 0));

    private final MechanismLigament2d setHumerus = pivot
            .append(new MechanismLigament2d("set humerus", Constants.Arm.HUMERUS_LENGTH, 0));
    private final MechanismLigament2d setManipulator = setHumerus
            .append(new MechanismLigament2d("set manipulator", Constants.Wrist.JOINT_TO_CORNER_DISTANCE, 0));

    public Arm() {
        shoulder = (RobotBase.isSimulation()) ? new SIMShoulder() : new Shoulder();
        wrist = (RobotBase.isSimulation()) ? new SIMWrist(this) : new Wrist(this);
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

        setHumerus.setAngle(getTargetShoulderAngle().minus(Rotation2d.fromDegrees(90)));
        setManipulator.setAngle(getTargetWristAngle().plus(Rotation2d.fromDegrees(90))
                .minus(getTargetShoulderAngle()));
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

    public void moveShoulderOffset(Rotation2d difference) {
        switch (gamePiece) {
            case Cone:
                target.shoulderConeOffset = Rotation2d
                        .fromRadians(target.shoulderConeOffset.getRadians() + difference.getRadians());
                break;
            case Cube:
                target.shoulderCubeOffset = Rotation2d
                        .fromRadians(target.shoulderCubeOffset.getRadians() + difference.getRadians());
        }
    }

    public void moveWristOffset(Rotation2d difference) {
        switch (gamePiece) {
            case Cone:
                target.wristConeOffset = Rotation2d
                        .fromRadians(target.wristConeOffset.getRadians() + difference.getRadians());
                break;
            case Cube:
                target.wristCubeOffset = Rotation2d
                        .fromRadians(target.wristCubeOffset.getRadians() + difference.getRadians());
        }
    }

    public void resetOffset() {
        switch (gamePiece) {
            case Cone:
                target.wristConeOffset = new Rotation2d();
                target.shoulderConeOffset = new Rotation2d();
                break;
            case Cube:
                target.wristCubeOffset = new Rotation2d();
                target.shoulderCubeOffset = new Rotation2d();
        }
    }

    public State getTarget() {
        return target;
    }

    public Rotation2d getTargetWristAngle() {
        return target.getWristAngleFor(this, gamePiece);
    }

    public Rotation2d getTargetShoulderAngle() {
        return target.getShoulderAngleFor(this.gamePiece);
    }

    public void setTargetFromNode(Node node) {
        switch (node.getLevel()) {
            case Low:
                this.target = State.Low;
                new ArmMoveCommand(this).schedule();
                break;
            case Mid:
                target = State.Mid;
                this.setGamePiece(node.getNodeType());
                break;
            case High:
                target = State.High;
                this.setGamePiece(node.getNodeType());
        }
    }

    public void setTarget(State target) {
        this.target = target;
        new ArmMoveCommand(this).schedule();
    }

    public GamePiece getGamePiece() {
        return gamePiece;
    }

    public void setGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
        this.rollers.setGamePiece(gamePiece);
        new ArmMoveCommand(this).schedule();
    }

    public void setRollerState(Rollers.State state) {
        this.rollerState = state;
        this.rollers.setRollerState(state);
        new ArmMoveCommand(this).schedule();
    }

    public Rollers.State getRollerState() {
        return this.rollerState;
    }

    public enum State {
        Teller(Constants.Shoulder.TELLER_CONE_ANGLE, Constants.Shoulder.TELLER_CUBE_ANGLE,
                Constants.Wrist.TELLER_CONE_ANGLE, Constants.Wrist.TELLER_CUBE_ANGLE),
        Slide(Constants.Shoulder.SLIDE_CONE_ANGLE, Constants.Shoulder.SLIDE_CUBE_ANGLE,
                Constants.Wrist.SLIDE_CONE_ANGLE, Constants.Wrist.SLIDE_CUBE_ANGLE),
        High(Constants.Shoulder.HIGH_CONE_ANGLE, Constants.Shoulder.HIGH_CUBE_ANGLE, Constants.Wrist.HIGH_CONE_ANGLE,
                Constants.Wrist.HIGH_CUBE_ANGLE),
        Mid(Constants.Shoulder.MID_CONE_ANGLE, Constants.Shoulder.MID_CUBE_ANGLE, Constants.Wrist.MID_CONE_ANGLE,
                Constants.Wrist.MID_CUBE_ANGLE),
        Low(Constants.Shoulder.INTAKE_CONE_ANGLE, Constants.Shoulder.LOW_CUBE_ANGLE, Constants.Wrist.INTAKE_CONE_ANGLE,
                Constants.Wrist.LOW_CUBE_ANGLE),
        Stowed(Constants.Shoulder.STOWED_ANGLE, Constants.Shoulder.STOWED_ANGLE, Constants.Wrist.STOWED_CONE_ANGLE,
                Constants.Wrist.STOWED_CUBE_ANGLE);

        private final Rotation2d shoulderCubeAngle;
        private final Rotation2d shoulderConeAngle;
        private final Rotation2d wristConeAngle;
        private final Rotation2d wristCubeAngle;

        private Rotation2d shoulderConeOffset = new Rotation2d();
        private Rotation2d shoulderCubeOffset = new Rotation2d();
        private Rotation2d wristConeOffset = new Rotation2d();
        private Rotation2d wristCubeOffset = new Rotation2d();

        State(Rotation2d shoulderConeAngle, Rotation2d shoulderCubeAngle, Rotation2d wristConeAngle,
                Rotation2d wristCubeAngle) {
            this.shoulderCubeAngle = shoulderCubeAngle;
            this.shoulderConeAngle = shoulderConeAngle;
            this.wristConeAngle = wristConeAngle;
            this.wristCubeAngle = wristCubeAngle;
        }

        public static State getTargetFromNode(Node node) {
            switch (node.getLevel()) {
                case Low:
                    return Low;
                case Mid:
                    return Mid;
                case High:
                    return High;
                default:
                    return Stowed;
            }
        }

        public Rotation2d getShoulderAngleFor(GamePiece gamePiece) {
            return (gamePiece == GamePiece.Cone) ? this.shoulderConeAngle.plus(this.shoulderConeOffset)
                    : this.shoulderCubeAngle.plus(this.shoulderCubeOffset);
        }

        public Rotation2d getWristAngleFor(Arm arm, GamePiece gamePiece) {
            if (this == Stowed && arm.rollerState == Rollers.State.Off) {
                return Constants.Wrist.LIMIT_SWITCH_OFFSET;
            }

            return (gamePiece == GamePiece.Cone) ? this.wristConeAngle.plus(this.wristConeOffset)
                    : this.wristCubeAngle.plus(this.wristCubeOffset);
        }

        public Node.Level closestLevel() {
            switch (this) {
                case Teller:
                case High:
                    return Node.Level.High;
                case Mid:
                    return Node.Level.Mid;
                default:
                    return Node.Level.Low;
            }
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
