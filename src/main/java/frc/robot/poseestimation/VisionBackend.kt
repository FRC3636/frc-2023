package frc.robot.poseestimation

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import java.util.*

abstract class VisionBackend {
    abstract val measurement: Optional<Measurement?>

    inner class Measurement(var timestamp: Double, var pose: Pose3d, var stdDeviation: Matrix<N3, N1>, var ambiguity: Double)
}