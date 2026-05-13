package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.util.PhoenixUtil;

/** TalonFX-backed IO implementation for the flywheel. */
public class FlywheelIOTalon implements FlywheelIO {
    private static final TalonFXConfiguration motorInitialConfigs = new TalonFXConfiguration()
            .withMotorOutput(
                    new MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Brake)

            )
            .withCurrentLimits(
                    new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(HoodConstants.kCurrentStatorLimit)
                            .withStatorCurrentLimitEnable(true))
            .withSlot0(
                    new Slot0Configs()
                            .withKP(HoodConstants.kP)
                            .withKI(HoodConstants.kI)
                            .withKD(HoodConstants.kD)

                            .withKS(HoodConstants.kS)
                            .withKV(HoodConstants.kV)
                            .withKA(HoodConstants.kA)

                            .withGravityType(GravityTypeValue.Arm_Cosine))

            .withMotionMagic(
                    new MotionMagicConfigs()
                            .withMotionMagicAcceleration(HoodConstants.MotionMagicAcceleration)
                            .withMotionMagicJerk(HoodConstants.MotionMagicJerk)

                            .withMotionMagicCruiseVelocity(HoodConstants.MotionMagicCruiseVelocity)
                            .withMotionMagicExpo_kV(HoodConstants.Expo_kV)
                            .withMotionMagicExpo_kA(HoodConstants.Expo_kA))

            .withFeedback(
                    new FeedbackConfigs()
                            .withSensorToMechanismRatio(50));
    private final CANBus kCANBus = HoodConstants.kCANBus;
    protected final TalonFX motor = new TalonFX(20, kCANBus);

    private final MotionMagicVelocityTorqueCurrentFOC m_controlRequest = new MotionMagicVelocityTorqueCurrentFOC(0.0);

    /* device status signals */
    private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity(false);
    private final StatusSignal<Current> motorTorqueCurrent = motor.getTorqueCurrent(false);

    public FlywheelIOTalon() {
        for (int i = 0; i < FlywheelConstants.kNumConfigAttempts; ++i) {
            var status = motor.getConfigurator().apply(motorInitialConfigs);
            if (status.isOK())
                break;
        }

        BaseStatusSignal.setUpdateFrequencyForAll(20.0, motorVelocity, motorTorqueCurrent);


        PhoenixUtil.registerSignals(false, motorVelocity, motorTorqueCurrent);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        var refresh = PhoenixUtil.refreshAll();
        inputs.angularVelocityRotationsPerSecond = motorVelocity.getValueAsDouble();
        inputs.currentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.motor_connected = refresh.isOK();
    }

    @Override
    public void setAngularSpeed(AngularVelocity angVelocity) {
        motor.setControl(m_controlRequest.withVelocity(angVelocity));
    }
}
