package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;

/**
 * Flywheel subsystem wrapper. Currently holds IO implementations and input
 * logging; control logic can be added as needed.
 */
public class Flywheel extends SubsystemBase {
    public FlywheelIO[] io;
    private final FlywheelIOInputsAutoLogged[] inputs;

    /** Creates a flywheel subsystem with one or more IO implementations. */
    public Flywheel(FlywheelIO... flywheelIOs) {
        this.io = flywheelIOs;

        // Init inputs
        this.inputs = new FlywheelIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new FlywheelIOInputsAutoLogged();
        }
    }

    
}
