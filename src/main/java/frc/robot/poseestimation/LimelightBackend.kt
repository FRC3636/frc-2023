package frc.robot.poseestimation

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.DoubleArraySubscriber
import edu.wpi.first.networktables.DoubleSubscriber
import edu.wpi.first.networktables.NetworkTableInstance
import frc.robot.Constants
import java.util.*

class LimelightBackend : VisionBackend() {
    private val botPose: DoubleArraySubscriber
    private val cl: DoubleSubscriber
    private val tl: DoubleSubscriber

    init {
        botPose = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(null)
        cl = table.getDoubleTopic("cl").subscribe(0.0)
        tl = table.getDoubleTopic("tl").subscribe(0.0)
    }

    override val measurement: Optional<Measurement?>
        get() {
            val updates = botPose.readQueue()
            if (updates.isEmpty()) {
                return Optional.empty<VisionBackend.Measurement?>() as Optional<VisionBackend.Measurement?>
            }
            val update = updates[updates.size - 1]
            if (Arrays.equals(update.value, DoubleArray(6))) {
                return Optional.empty<VisionBackend.Measurement?>() as Optional<VisionBackend.Measurement?>
            }
            val x = update.value[0]
            val y = update.value[1]
            val z = update.value[2]
            val roll = Units.degreesToRadians(update.value[3])
            val pitch = Units.degreesToRadians(update.value[4])
            val yaw = Units.degreesToRadians(update.value[5])
            val latency = cl.get() + tl.get()
            val timestamp = update.timestamp * 1e-6 - latency * 1e-3
            val pose = Pose3d(Translation3d(x, y, z), Rotation3d(roll, pitch, yaw))
            return Optional.of<VisionBackend.Measurement?>(Measurement(
                    timestamp,
                    pose,
                    Constants.VisionConstants.LIMELIGHT_STD_DEV,
                    0.0
            )) as Optional<VisionBackend.Measurement?>
        }

    companion object {
        private val table = NetworkTableInstance.getDefault().getTable("limelight")
    }
}