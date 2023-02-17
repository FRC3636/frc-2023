package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.Constants;

public class LimelightBackend extends VisionBackend {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private DoubleArraySubscriber botPose;
    private DoubleSubscriber cl;
    private DoubleSubscriber tl;

    public LimelightBackend() {
        botPose = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(null);
        cl = table.getDoubleTopic("cl").subscribe(0);
        tl = table.getDoubleTopic("tl").subscribe(0);
    }


    @Override
    public Optional<VisionBackend.Measurement> getMeasurement() {
        TimestampedDoubleArray[] updates = botPose.readQueue();

        if (updates.length == 0) {
            return Optional.empty();
        }

        TimestampedDoubleArray update = updates[updates.length - 1];

        if (update.value == new double[6]) {
            return Optional.empty();
        }

        double x = update.value[0];
        double y = update.value[1];
        double z = update.value[2];
        double roll = Units.degreesToRadians(update.value[3]);
        double pitch = Units.degreesToRadians(update.value[4]);
        double yaw = Units.degreesToRadians(update.value[5]);

        double latency = cl.get() + tl.get();

        double timestamp = (update.timestamp * 1e-6) - (latency * 1e-3);
        Pose3d pose = new Pose3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));

        return Optional.of(new Measurement(
            timestamp,
            pose,
            Constants.Vision.LIMELIGHT_STD_DEV
        ));
    }
}
