package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for a swerve module. Implementations should populate
 * {@link ModuleIOInputs} and apply outputs.
 */
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

    /** Updates the inputs structure with the latest sensor values. */
    public default void updateInputs(ModuleIOInputs inputs) {
        // Default implementation does nothing
    }

    /** Sets open-loop drive output (typically percent or volts depending on IO). */
    public default void setDriveOpenLoop(double output) {
        // Default implementation does nothing
    }

    /** Sets open-loop steering output (typically percent or volts depending on IO). */
    public default void setTurnOpenLoop(double output) {
        // Default implementation does nothing
    }

    /** Sets closed-loop drive velocity in radians/sec. */
    public default void setDriveVelocity(double velocityRadPerSec) {
        // Default implementation does nothing
    }

    /** Sets closed-loop steering position. */
    public default void setTurnPosition(Rotation3d position) {
        // Default implementation does nothing
    }
}
