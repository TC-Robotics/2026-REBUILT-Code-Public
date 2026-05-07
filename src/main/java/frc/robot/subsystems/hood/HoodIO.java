package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public boolean connected = false;
        public double currentAmps;
        public double hoodAngle;
    }

    public default void updateInputs(HoodIOInputs inputs) {
    }

    public default void setHoodAngle(Angle angle) {
    }

    public default void zero() {
    }
}
