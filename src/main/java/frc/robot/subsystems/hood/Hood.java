package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(HoodIO io) {
        this.io = io;
        Logger.processInputs("Hood", inputs);
        /* set the default command to neutral output */

        // setDefaultCommand(manualDrive(() -> 0.0));

        /* alternatively, the default command can hold position */
        setDefaultCommand(holdPosition());

    }

    public Command holdPosition() {
        return runOnce(() -> io.setHoodAngle(Angle.ofBaseUnits(inputs.hoodAngle, Degrees)));
    }
}
