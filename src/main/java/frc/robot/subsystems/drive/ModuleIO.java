package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public Rotation3d turnAbsolutePosition = Rotation3d.kZero;
        public Rotation3d turnPosition = Rotation3d.kZero;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation3d[] odometryTurnPositions = new Rotation3d[] {};

    }

    public default void updateInputs(ModuleIOInputs inputs) {
        // Default implementation does nothing
    }

    public default void setDriveOpenLoop(double output) {
        // Default implementation does nothing
    }

    public default void setTurnOpenLoop(double output) {
        // Default implementation does nothing
    }

    public default void setDriveVelocity(double velocityRadPerSec) {
        // Default implementation does nothing
    }

    public default void setTurnPosition(Rotation3d position) {
        // Default implementation does nothing
    }
}
