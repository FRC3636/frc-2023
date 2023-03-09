package frc.robot.poseestimation

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import frc.robot.Constants
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import java.util.*

class PhotonVisionBackend : VisionBackend() {
    private val camera: PhotonCamera
    private val poseEstimator: PhotonPoseEstimator

    init {
        camera = PhotonCamera("arducam")
        camera.driverMode = false
        camera.pipelineIndex = 0
        val fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile)
        poseEstimator = PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, camera, Constants.VisionConstants.PHOTONVISION_TRANSFORM)
    }

    override val measurement: Optional<Measurement?>
        get() = poseEstimator.update().map { result: EstimatedRobotPose ->
            Measurement(
                    result.timestampSeconds,
                    result.estimatedPose,
                    Constants.VisionConstants.PHOTONVISION_STD_DEV,
                    result.targetsUsed[0].poseAmbiguity
            )
        }
}