package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

/** Hardware abstraction for flywheel control. */
public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public boolean motor_connected;
        public double currentAmps;
        public double angularVelocityRotationsPerSecond;
    }

    /** Updates the inputs structure with the latest sensor readings. */
    public default void updateInputs(FlywheelIOInputs inputs) {

    }

    /** Sets the target angular velocity of the flywheel. */
    public default void setAngularSpeed(AngularVelocity angVelocity) {

    }

}
