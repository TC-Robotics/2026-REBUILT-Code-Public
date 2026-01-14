package frc.robot.subsystems.hood;

public interface HoodIO {
    public static class HoodIOInputs {
        public boolean connected = false;
        public double currentAmps;
        public double angularVelocityRotationsPerSecond;
        public double positionRotations;
    }
}
