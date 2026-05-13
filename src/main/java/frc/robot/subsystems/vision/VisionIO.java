package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;


/**
 * Hardware abstraction for vision pipelines. Implementations populate the
 * {@link VisionIOInputs} structure with camera measurements.
 */
public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        /** Latest yaw/pitch observation from the camera. */
        public TargetObservation latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
        /** Pose observations from the vision pipeline (may be empty). */
        public PoseObservation[] poseObservations = new PoseObservation[0];
        /** Tag IDs detected in the current frame. */
        public int[] tagIds = new int[0];
    }    

    /** Simple target observation (yaw/pitch) relative to the camera. */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    /** Full pose observation with quality metrics and metadata. */
    public static record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double averageTagDistance,
        PoseObservationType type) {}
    
    /** Origin of the pose estimate. */
    public static enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    /** Updates the provided inputs structure with the latest data. */
    public default void updateInputs(VisionIOInputs inputs) {}
}
