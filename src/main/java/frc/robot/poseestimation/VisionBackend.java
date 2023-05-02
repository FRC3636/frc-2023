package frc.robot.poseestimation;

import java.util.Optional;

public abstract class VisionBackend {
    public abstract Optional<PoseEstimation.Measurement> getMeasurement();
}
