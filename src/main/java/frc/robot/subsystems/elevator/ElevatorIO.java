package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public interface ElevatorIO {
    public static class ElevatorIOInputs {
        public boolean connected = false;
        public double appliedVolts;
        public double currentAmps;
        public double positionRotations;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void zero() {
    };

    public default void setPosition(double positionMetres) {
    };

    public default double positionRotationsToMetres(double rotations) {
        return rotations;
    }

    public default double positionMetresToRotations(double metres) {
        return metres;
    }
}
