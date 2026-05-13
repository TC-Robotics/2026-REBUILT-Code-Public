package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Hood subsystem controlling the shooter hood angle. */
public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    /** Creates a hood subsystem with the given IO implementation. */
    public Hood(HoodIO io) {
        this.io = io;
        Logger.processInputs("Hood", inputs);
        /* set the default command to neutral output */

        // setDefaultCommand(manualDrive(() -> 0.0));

        /* alternatively, the default command can hold position */
        setDefaultCommand(holdPosition());

    }

    /** Holds the hood at the current angle using closed-loop control. */
    public Command holdPosition() {
        return runOnce(() -> io.setHoodAngle(Angle.ofBaseUnits(inputs.hoodAngle, Degrees)));
    }
}
