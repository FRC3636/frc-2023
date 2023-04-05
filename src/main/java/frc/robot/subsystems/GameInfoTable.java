package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utils.GamePiece;

import java.util.Objects;
import java.util.function.Function;

// used by lights
public class GameInfoTable extends SubsystemBase {
    private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private final NetworkTable table = instance.getTable("GameInfo");
    private final CachedVal<GameStage> stage = new CachedVal<>(
            "stage",
            GameStage.Disabled,
            gs -> NetworkTableValue.makeString(gs.name().toLowerCase())
    );
    private final CachedVal<DriverStation.Alliance> alliance = new CachedVal<>(
            "alliance",
            DriverStation.Alliance.Invalid,
            al -> NetworkTableValue.makeString(al.name().toLowerCase())
    );
    private final CachedVal<GamePiece> piece = new CachedVal<>(
            "piece",
            GamePiece.Cone,
            gp -> NetworkTableValue.makeString(gp.name().toLowerCase())
    );
    private final CachedVal<DriverStation.MatchType> matchType = new CachedVal<>(
            "matchtype",
            DriverStation.MatchType.None,
            mt -> NetworkTableValue.makeString(mt.name().toLowerCase())
    );
    private final NTVal<Double> time = new NTVal<>(
            "time",
            0d,
            NetworkTableValue::makeDouble
    );
    private final CachedVal<Boolean> estopped = new CachedVal<>(
            "estopped",
            false,
            NetworkTableValue::makeBoolean
    );
    private final CachedVal<Boolean> holding = new CachedVal<>(
            "holding",
            false,
            NetworkTableValue::makeBoolean
    );

    public GameInfoTable() {}

    @Override
    public void periodic() {
        this.piece.setValue(RobotContainer.arm.getGamePiece());
        this.alliance.setValue(DriverStation.getAlliance());
        this.matchType.setValue(DriverStation.getMatchType());
        this.estopped.setValue(DriverStation.isEStopped());
        this.holding.setValue(RobotContainer.arm.getRollers().isHoldingGamePiece());

        this.time.setValue(DriverStation.getMatchTime());
    }

    public void setStage(GameStage s) {
        stage.setValue(s);
    }

    public enum GameStage {
        Auto,
        TeleOp,
        Test,
        Disabled
    }

    private class NTVal<T> {
        protected T value = null;
        protected String name;
        protected Function<T, NetworkTableValue> valueFunction;

        public NTVal(String name, T defaultValue, Function<T, NetworkTableValue> valueFunction) {
            this.name = name;
            this.valueFunction = valueFunction;
            this.setValue(defaultValue);
        }

        public T getValue() {
            return value;
        }

        public void setValue(T value) {
            table.putValue(this.name, this.valueFunction.apply(value));
        }
    }

    private class CachedVal<T> extends NTVal<T> {
        public CachedVal(String name, T defaultValue, Function<T, NetworkTableValue> function) {
            super(name, defaultValue, function);
        }

        @Override
        public void setValue(T value) {
            if(!Objects.equals(this.value, value)) {
                super.setValue(value);
            }
            this.value = value;
        }
    }
}
