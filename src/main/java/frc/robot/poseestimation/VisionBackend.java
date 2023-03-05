package frc.robot.poseestimation;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public abstract class VisionBackend {
    public abstract Optional<Measurement> getMeasurement();

    public class Measurement {
        public double timestamp;
        public Pose3d pose;
        public Matrix<N3, N1> stdDeviation;
        public double ambiguity;

        public Measurement(double timestamp, Pose3d pose, Matrix<N3, N1> stdDeviation, double ambiguity) {
            this.timestamp = timestamp;
            this.pose = pose;
            this.stdDeviation = stdDeviation;
            this.ambiguity = ambiguity;
        }
    }
}
