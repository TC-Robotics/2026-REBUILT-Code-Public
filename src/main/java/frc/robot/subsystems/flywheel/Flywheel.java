package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;

public class Flywheel extends SubsystemBase {
    public FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }
}
