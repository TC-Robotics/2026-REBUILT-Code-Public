package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation3d yawPosition = Rotation3d.kZero;
        public double yawVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation3d[] odometryYawPositions = new Rotation3d[] {};

    }

    public default void updateInputs(GyroIOInputs inputs) {
        // Default implementation does nothing
    }
}
