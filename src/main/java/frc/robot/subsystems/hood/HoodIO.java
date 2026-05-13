package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;;

/** Hardware abstraction for hood angle control. */
public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public boolean connected = false;
        public double currentAmps;
        public double hoodAngle;
    }

    /** Updates the inputs structure with the latest sensor readings. */
    public default void updateInputs(HoodIOInputs inputs) {
    }

    /** Sets the target hood angle. */
    public default void setHoodAngle(Angle angle) {
    }

    /** Zeros the hood position reference. */
    public default void zero() {
    }
}
