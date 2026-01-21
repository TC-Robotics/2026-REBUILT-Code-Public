package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public boolean connected = false;
        public double currentAmps;
        public double angularVelocityRotationsPerSecond;
        public double positionRotations;
    }
}
