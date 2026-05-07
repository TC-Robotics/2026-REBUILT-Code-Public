package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public boolean motor_connected;
        public double currentAmps;
        public double angularVelocityRotationsPerSecond;
    }

    public default void updateInputs(FlywheelIOInputs inputs) {

    }

    public default void setAngularSpeed(AngularVelocity angVelocity) {

    }

}
