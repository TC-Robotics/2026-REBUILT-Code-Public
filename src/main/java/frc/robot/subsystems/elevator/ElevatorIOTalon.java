package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalon implements ElevatorIO {

    /** Configs common across all motors. */
    private static final TalonFXConfiguration motorInitialConfigs = new TalonFXConfiguration()
            .withMotorOutput(
                    new MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(ElevatorConstants.kCurrentStatorLimit)
                            .withStatorCurrentLimitEnable(true));

    /** Configs common across just the leader motors. */
    private static final TalonFXConfiguration leaderInitialConfigs = motorInitialConfigs.clone()
            .withFeedback(
                    motorInitialConfigs.Feedback.clone()
                            .withSensorToMechanismRatio(8))
            .withSlot0(
                    motorInitialConfigs.Slot0.clone()
                            .withKP(ElevatorConstants.kP)
                            .withKI(ElevatorConstants.kI)
                            .withKD(ElevatorConstants.kD)
                            .withKS(ElevatorConstants.kS)
                            .withKV(ElevatorConstants.kV)
                            .withKA(ElevatorConstants.kA)
                            .withKG(ElevatorConstants.kG)
                            .withGravityType(GravityTypeValue.Elevator_Static))
            .withMotionMagic(
                    motorInitialConfigs.MotionMagic.clone()
                            .withMotionMagicCruiseVelocity(ElevatorConstants.MotionMagicCruiseVelocity)
                            .withMotionMagicAcceleration(ElevatorConstants.MotionMagicAcceleration)
                            .withMotionMagicJerk(ElevatorConstants.MotionMagicJerk)
                            .withMotionMagicExpo_kV(ElevatorConstants.Expo_kV)
                            .withMotionMagicExpo_kA(ElevatorConstants.Expo_kA))
            .withSoftwareLimitSwitch(
                    motorInitialConfigs.SoftwareLimitSwitch.clone()
                            .withReverseSoftLimitThreshold(Rotations.of(0))
                            .withReverseSoftLimitEnable(true));

    /** Configs for {@link #leaderMotor}. */
    private final TalonFXConfiguration leaderMotorConfigs = leaderInitialConfigs.clone()
            .withMotorOutput(
                    leaderInitialConfigs.MotorOutput.clone()
                            .withInverted(InvertedValue.CounterClockwise_Positive));

    /** Configs for {@link #followerMotor}. */
    private final TalonFXConfiguration followerMotorConfigs = motorInitialConfigs.clone()
            .withMotorOutput(
                    motorInitialConfigs.MotorOutput.clone()
                            .withInverted(InvertedValue.CounterClockwise_Positive));

    /* leader and follower motors */
    private final CANBus kCANBus = ElevatorConstants.kCANBus;
    protected final TalonFX leaderMotor = new TalonFX(0, kCANBus);
    protected final TalonFX followerMotor = new TalonFX(1, kCANBus);

    /* device status signals */
    private final StatusSignal<Angle> leaderMotorPosition = leaderMotor.getPosition(false);
    private final StatusSignal<AngularVelocity> leaderMotorVelocity = leaderMotor.getVelocity(false);
    private final StatusSignal<Current> leaderMotorTorqueCurrent = leaderMotor.getTorqueCurrent(false);

    /* controls used by the leader motors */
    private final MotionMagicExpoTorqueCurrentFOC setpointRequest = new MotionMagicExpoTorqueCurrentFOC(0);
    private final DutyCycleOut manualRequest = new DutyCycleOut(0);
    private final DutyCycleOut calibrationRequest = new DutyCycleOut(-0.1)
            .withIgnoreHardwareLimits(true)
            .withIgnoreSoftwareLimits(true);

    /** Trigger to detect when the elevator drives into a hard stop. */
    public final Trigger isHardStop = new Trigger(() -> {
        return leaderMotorVelocity.getValue().abs(RotationsPerSecond) < 1 &&
                leaderMotorTorqueCurrent.getValue().abs(Amps) > 10;
    }).debounce(0.1);

    ElevatorIOTalon() {
        for (int i = 0; i < ElevatorConstants.kNumConfigAttempts; ++i) {
            var status = leaderMotor.getConfigurator().apply(leaderMotorConfigs);
            if (status.isOK())
                break;
        }
        for (int i = 0; i < ElevatorConstants.kNumConfigAttempts; ++i) {
            var status = followerMotor.getConfigurator().apply(followerMotorConfigs);
            if (status.isOK())
                break;
        }

        BaseStatusSignal.setUpdateFrequencyForAll(20.0, leaderMotorPosition, leaderMotorVelocity, leaderMotorTorqueCurrent);

        followerMotor.setControl(
                new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));

        PhoenixUtil.registerSignals(false, leaderMotorPosition, leaderMotorVelocity, leaderMotorTorqueCurrent);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        /* refresh all status signals */
        PhoenixUtil.refreshAll();
    }
}
