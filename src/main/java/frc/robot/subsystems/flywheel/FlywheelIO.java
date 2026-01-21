package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public boolean connected = false;
        public double currentAmps;
        public double angularVelocityRotationsPerSecond;
        public double flywhellEjectionSpeedMetresPerSecond;
    }
}
