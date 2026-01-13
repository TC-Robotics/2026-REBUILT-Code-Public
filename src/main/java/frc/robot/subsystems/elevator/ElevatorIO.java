package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meter;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean connected = false;
        public double currentAmps;
        public double positionRotations;
        public double angularVelocityRotationsPerSecond;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void zero() {
    }

    public default void setPosition(double positionMetres) {
    }

    public default void manualDrive(double voltageProportion) {}

    public default double positionRotationsToMetres(double rotations) {
        return rotations * Math.PI * 2 * ElevatorConstants.kDrumRadius.in(Meter) / ElevatorConstants.kGearRatio;
    }

    public default double positionMetresToRotations(double metres) {
        return metres / (Math.PI * 2 * ElevatorConstants.kDrumRadius.in(Meter)) * ElevatorConstants.kGearRatio;
    }

    public default void calibrateDrive() {}
}
