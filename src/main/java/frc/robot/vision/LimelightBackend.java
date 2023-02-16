package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.Constants;

public class LimelightBackend extends VisionBackend {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private DoubleArraySubscriber botPose;
    private DoubleSubscriber cl;
    private DoubleSubscriber tl;

    public LimelightBackend() {
        botPose = table.getDoubleArrayTopic("botpose").subscribe(null);
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

        double x = update.value[0];
        double y = update.value[1];
        double z = update.value[2];
        double roll = update.value[3];
        double pitch = update.value[4];
        double yaw = update.value[5];

        double latency = cl.get() + tl.get();

        return Optional.of(new Measurement(
            (update.timestamp / 1e6) - (latency / 1e3),
            new Pose3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw)), 
            Constants.Vision.LIMELIGHT_STD_DEV
        ));
    }
}
