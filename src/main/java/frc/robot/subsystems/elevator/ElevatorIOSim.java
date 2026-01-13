package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOSim extends ElevatorIOTalon {

    /* simulation */
    private final ElevatorSim elevatorSim_leaderMotor = new ElevatorSim(
            DCMotor.getKrakenX60Foc(2),
            ElevatorConstants.kGearRatio, 5, ElevatorConstants.kDrumRadius.in(Meters),
            ElevatorConstants.kMinHeight.in(Meters), ElevatorConstants.kMaxHeight.in(Meters), true,
            ElevatorConstants.kMinHeight.in(Meters));

    private Notifier simNotifier = null;
    private double lastSimTime = 0.0;

    public ElevatorIOSim() {
        super();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        super.updateInputs(inputs);
    }

    private void startSimThread() {
        leaderMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        followerMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;

        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            /* Calculate the time delta */
            final double currentTime = Utils.getCurrentTimeSeconds();
            final double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            final var leaderMotorSim = leaderMotor.getSimState();
            final var followerMotorSim = followerMotor.getSimState();

            /* First set the supply voltage of all the devices */
            leaderMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
            followerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

            /* Then calculate the new position and velocity of the simulated elevator */
            elevatorSim_leaderMotor.setInputVoltage(leaderMotorSim.getMotorVoltage());
            elevatorSim_leaderMotor.update(deltaTime);

            /*
             * Apply the new rotor position and velocity to the motors (before gear ratio)
             */
            leaderMotorSim.setRawRotorPosition(
                    Radians.of(elevatorSim_leaderMotor.getPositionMeters()
                            / ElevatorConstants.kDrumRadius.in(Meters)
                            * ElevatorConstants.kGearRatio));
            leaderMotorSim.setRotorVelocity(
                    RadiansPerSecond.of(elevatorSim_leaderMotor.getVelocityMetersPerSecond()
                            / ElevatorConstants.kDrumRadius.in(Meters)
                            * ElevatorConstants.kGearRatio));
            followerMotorSim.setRawRotorPosition(
                    Radians.of(elevatorSim_leaderMotor.getPositionMeters()
                            / ElevatorConstants.kDrumRadius.in(Meters)
                            * ElevatorConstants.kGearRatio));
            followerMotorSim.setRotorVelocity(
                    RadiansPerSecond.of(elevatorSim_leaderMotor.getVelocityMetersPerSecond()
                            / ElevatorConstants.kDrumRadius.in(Meters)
                            * ElevatorConstants.kGearRatio));
        });
        simNotifier.startPeriodic(ElevatorConstants.kSimLoopPeriod);
    }
}
