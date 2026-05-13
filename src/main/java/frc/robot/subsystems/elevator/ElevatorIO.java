package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meter;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for the elevator. Implementations should populate
 * {@link ElevatorIOInputs} and apply control outputs.
 */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean connected = false;
        public double currentAmps;
        public double positionRotations;
        public double angularVelocityRotationsPerSecond;
    }

    /** Updates the inputs structure with the latest sensor data. */
    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    /** Zeros the elevator encoder position. */
    public default void zero() {
    }

    /** Sets the desired elevator position in meters. */
    public default void setPosition(double positionMetres) {
    }

    /** Applies a manual duty-cycle output to the elevator motors. */
    public default void manualDrive(double voltageProportion) {}

    /** Converts sensor rotations to linear elevator extension in meters. */
    public default double positionRotationsToMetres(double rotations) {
        return rotations * Math.PI * 2 * ElevatorConstants.kDrumRadius.in(Meter) / ElevatorConstants.kGearRatio;
    }

    /** Converts linear elevator extension in meters to sensor rotations. */
    public default double positionMetresToRotations(double metres) {
        return metres / (Math.PI * 2 * ElevatorConstants.kDrumRadius.in(Meter)) * ElevatorConstants.kGearRatio;
    }

    /** Drives slowly toward the hard stop to calibrate zero. */
    public default void calibrateDrive() {}
}
