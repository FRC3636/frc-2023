package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmMoveCommand;

public class Arm extends SubsystemBase {

    private final Shoulder shoulder;
    private final Wrist wrist;
    private final Rollers rollers;

    private final Mechanism2d arm = new Mechanism2d(3, Constants.Arm.PIVOT_HEIGHT + 0.5);
    private final MechanismRoot2d pivot = arm.getRoot("pivot", 1, Constants.Arm.PIVOT_HEIGHT);
    private final MechanismLigament2d humerus = pivot.append(new MechanismLigament2d("humerus", Constants.Arm.HUMERUS_LENGTH, 0));
    private final MechanismLigament2d manipulator = humerus.append(new MechanismLigament2d("manipulator", Constants.Wrist.JOINT_TO_CORNER_DISTANCE, 0));

    private final MechanismLigament2d setHumerus = pivot.append(new MechanismLigament2d("set humerus", Constants.Arm.HUMERUS_LENGTH, 0));
    private final MechanismLigament2d setManipulator = setHumerus.append(new MechanismLigament2d("set manipulator", Constants.Wrist.JOINT_TO_CORNER_DISTANCE, 0));

    public Arm() {
        shoulder = new Shoulder(this);
        wrist = new Wrist(this);
        rollers = new Rollers();

        humerus.setLineWeight(20);

        setHumerus.setColor(new Color8Bit(0, 255, 255));
        setManipulator.setColor(new Color8Bit(0, 100, 255));

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
        High(Constants.Shoulder.HIGH_CONE_ANGLE, Constants.Shoulder.HIGH_CUBE_ANGLE, Constants.Wrist.HIGH_CONE_ANGLE, Constants.Wrist.HIGH_CUBE_ANGLE),
        Mid(Constants.Shoulder.MID_CONE_ANGLE, Constants.Shoulder.MID_CUBE_ANGLE, Constants.Wrist.MID_CONE_ANGLE, Constants.Wrist.MID_CUBE_ANGLE),
        Low(Constants.Shoulder.INTAKE_CONE, Constants.Shoulder.INTAKE_CONE, Constants.Wrist.INTAKE_CONE, Constants.Wrist.MID_CUBE_ANGLE),
        Stowed(Constants.Shoulder.STOWED_ANGLE, Constants.Shoulder.STOWED_ANGLE, Constants.Wrist.STOWED_ANGLE, Constants.Wrist.CUBE_ANGLE);

        private final Rotation2d shoulderCubeAngle;
        private final Rotation2d shoulderConeAngle;
        private final Rotation2d wristConeAngle;
        private final Rotation2d wristCubeAngle;
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

        public Rotation2d getShoulderAngle() {
            return(gamePiece == GamePiece.Cone) ? shoulderConeAngle : shoulderCubeAngle;
        }

        public Rotation2d getWristAngle() {
            if(this == State.Stowed && rollerState == Rollers.State.Off) {
                return Constants.Wrist.LIMIT_SWITCH_OFFSET;
            }
            return(gamePiece == GamePiece.Cone) ? wristConeAngle : wristCubeAngle;
        }

        public Rotation2d defaultWristAngle(GamePiece gamePiece) {
            return (gamePiece == GamePiece.Cone)? Constants.Wrist.CONE_ANGLE : Constants.Wrist.CUBE_ANGLE;
        }

        private static State target = State.Stowed;
        private static GamePiece gamePiece = GamePiece.Cube;
        private static Rollers.State rollerState = Rollers.State.Off;

        public static State getTarget() {
            return target;
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
    }
}
