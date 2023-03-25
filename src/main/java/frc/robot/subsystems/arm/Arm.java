package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
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

import java.util.Optional;

public class Arm extends SubsystemBase {

    private State target = State.Stowed;
    private GamePiece gamePiece = GamePiece.Cube;
    private Rollers.State rollerState = Rollers.State.Off;

    private final Shoulder shoulder;
    private final Wrist wrist;

    private Rotation2d temporaryAngleOffset = new Rotation2d();

    private final Rollers rollers;

    private final Mechanism2d arm = new Mechanism2d(Units.inchesToMeters(28), Constants.Arm.PIVOT_HEIGHT + 0.5);

    private final MechanismRoot2d pivot = arm.getRoot("pivot", Units.inchesToMeters(22), Constants.Arm.PIVOT_HEIGHT);
    private final MechanismLigament2d humerus = pivot
            .append(new MechanismLigament2d("humerus", Constants.Arm.HUMERUS_LENGTH , 0));
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
        RobotContainer.armTab.addDouble("Manipulator Height", this::getManipulatorHeight);
        RobotContainer.armTab.addDouble("Wrist Height", this::getWristHeight);
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

        return new double[]{
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

    public Rollers getRollers() {
        return rollers;
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

    public void moveHeightOffset(double difference) {
        switch (gamePiece) {
            case Cone:
                target.heightConeOffset += difference;
                break;
            case Cube:
                target.heightCubeOffset += difference;
        }
    }

    public void setTemporaryAngleOffset(Rotation2d temporaryAngleOffset) {
        this.temporaryAngleOffset = temporaryAngleOffset;
    }

    public void resetTemporaryAngleOffset() {
        this.temporaryAngleOffset = new Rotation2d();
    }

    public void resetOffset() {
        switch (gamePiece) {
            case Cone:
                target.heightConeOffset = 0;
                target.shoulderConeOffset = new Rotation2d();
                break;
            case Cube:
                target.heightCubeOffset = 0;
                target.shoulderCubeOffset = new Rotation2d();
        }
    }

    public State getTarget() {
        return target;
    }

    public Rotation2d getShoulderAngleFromState(State state) {
        if(state.getShoulderOverride().isPresent()) {
            return state.getShoulderOverride().get().plus(temporaryAngleOffset);
        }
        return Shoulder.getShoulderAngleFromHeight(
                state.getHeightFor(this.gamePiece),
                state.getWristAngleFor(this.gamePiece).minus(temporaryAngleOffset)
        );
    }

    public Rotation2d getWristAngleFromState(State state) {
        if(target.getShoulderOverride().isPresent()) {
            if(rollerState == Rollers.State.Off) {
                return target.getWristAngleFor(gamePiece);
            }
            return Wrist.getWristAngleFromHeight(state.getHeightFor(gamePiece), state.getShoulderOverride().get().plus(temporaryAngleOffset));
        }
        return target.getWristAngleFor(gamePiece).minus(temporaryAngleOffset);
    }

    public Rotation2d getTargetWristAngle() {
        return getWristAngleFromState(target);
    }


    public Rotation2d getTargetShoulderAngle() {
        return getShoulderAngleFromState(target);
    }

    public double getTargetHeight() {
        return target.getHeightFor(gamePiece);
    }

    public void setIdleMode(CANSparkMax.IdleMode idleMode) {
        shoulder.setIdleMode(idleMode);
        wrist.setIdleMode(idleMode);
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

    public double getManipulatorHeight() {
        return Constants.Arm.PIVOT_HEIGHT - (Constants.Arm.HUMERUS_LENGTH * getShoulderAngle().getCos() - Constants.Arm.MANIPULATOR_LENGTH * getWristAngle().getSin());
    }

    public double getWristHeight() {
        return Constants.Arm.PIVOT_HEIGHT - (Constants.Arm.HUMERUS_LENGTH * getShoulderAngle().getCos());
    }

    public Rollers.State getRollerState() {
        return this.rollerState;
    }

    public enum State {
        Teller(Constants.Wrist.STANDING_CONE_INTAKE_ANGLE, Constants.Wrist.CUBE_INTAKE_ANGLE,
                Constants.Arm.TELLER_CONE_HEIGHT, Constants.Arm.TELLER_CUBE_HEIGHT),

        Slide(Constants.Wrist.SLIDE_CONE_ANGLE, Constants.Wrist.SLIDE_CUBE_ANGLE,
                Constants.Arm.SLIDE_CONE_HEIGHT, Constants.Arm.SLIDE_CUBE_HEIGHT),

        High(Constants.Wrist.SCORING_CONE_ANGLE, Constants.Wrist.SCORING_CUBE_ANGLE,
                Constants.Arm.HIGH_CONE_HEIGHT, Constants.Arm.HIGH_CUBE_HEIGHT),

        Mid(Constants.Wrist.SCORING_CONE_ANGLE, Constants.Wrist.SCORING_CUBE_ANGLE,
                Constants.Arm.MID_CONE_HEIGHT, Constants.Arm.MID_CUBE_HEIGHT),

        Low(Constants.Wrist.STANDING_CONE_INTAKE_ANGLE, Constants.Wrist.CUBE_INTAKE_ANGLE,
                Constants.Arm.LOW_CONE_HEIGHT, Constants.Arm.LOW_CUBE_HEIGHT),

        Stowed(Constants.Shoulder.STOWED_ANGLE, Constants.Wrist.LIMIT_SWITCH_OFFSET, Constants.Wrist.LIMIT_SWITCH_OFFSET,
                Constants.Arm.STOWED_CONE_HEIGHT, Constants.Arm.STOWED_CUBE_HEIGHT);

        private final double coneHeight;
        private final double cubeHeight;
        private final Rotation2d wristCubeAngle;
        private final Rotation2d wristConeAngle;
        private final Optional<Rotation2d> shoulderOverride;

        private Rotation2d shoulderConeOffset = new Rotation2d();
        private Rotation2d shoulderCubeOffset = new Rotation2d();
        private double heightConeOffset = 0;
        private double heightCubeOffset = 0;

        State(Rotation2d wristConeAngle, Rotation2d wristCubeAngle, double ConeHeight,
              double CubeHeight) {
            this.wristCubeAngle = wristCubeAngle;
            this.wristConeAngle = wristConeAngle;
            this.coneHeight = ConeHeight;
            this.cubeHeight = CubeHeight;
            shoulderOverride = Optional.empty();
        }

        State(Rotation2d shoulderOverride, Rotation2d wristConeAngle, Rotation2d wristCubeAngle, double coneHeight, double cubeHeight) {
            this.coneHeight = coneHeight;
            this.cubeHeight = cubeHeight;
            this.shoulderOverride = Optional.of(shoulderOverride);
            this.wristConeAngle = wristConeAngle;
            this.wristCubeAngle = wristCubeAngle;
        }

        public Optional<Rotation2d> getShoulderOverride() {
            return shoulderOverride;
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

        public double getHeightFor(GamePiece piece) {
            return piece == GamePiece.Cone ? this.coneHeight + this.heightConeOffset : this.cubeHeight + this.heightCubeOffset;
        }

        public Rotation2d getWristAngleFor(GamePiece gamePiece) {
            return (gamePiece == GamePiece.Cone) ?
                    this.wristConeAngle.plus(this.shoulderConeOffset)
                    : this.wristCubeAngle.plus(this.shoulderCubeOffset);
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
                    value.coneHeight + value.heightConeOffset,
                    value.cubeHeight + value.heightCubeOffset,
                    value.wristConeAngle.plus(value.shoulderConeOffset).getRadians(),
                    value.wristCubeAngle.plus(value.shoulderCubeOffset).getRadians(),
            };
        }
    }
}
