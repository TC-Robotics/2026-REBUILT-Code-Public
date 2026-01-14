package frc.robot.subsystems.flywheel;

public interface FlywheelIO {
    public static class FlywheelIOInputs {
        public static class HoodIOInputs {
            public boolean connected = false;
            public double currentAmps;
            public double angularVelocityRotationsPerSecond;
            public double flywhellEjectionSpeedMetresPerSecond;
        }
    }
}
