package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public boolean[] motors_connected;
        public double currentAmps;
        public double angularVelocityRotationsPerSecond;
        public double flywhellEjectionSpeedMetresPerSecond;
    }
}
