package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.PhoenixUtil;


public class HoodIOTalon implements HoodIO {
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

                .withGravityType(GravityTypeValue.Arm_Cosine)
        )

        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(HoodConstants.MotionMagicAcceleration)
                .withMotionMagicJerk(HoodConstants.MotionMagicJerk)

                .withMotionMagicCruiseVelocity(HoodConstants.MotionMagicCruiseVelocity)
                .withMotionMagicExpo_kV(HoodConstants.Expo_kV)
                .withMotionMagicExpo_kA(HoodConstants.Expo_kA)
        )
        
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(50)   
        );

    private final CANBus kCANBus = HoodConstants.kCANBus;
    protected final TalonFX motor = new TalonFX(20, kCANBus);

    private final MotionMagicExpoTorqueCurrentFOC m_ControlRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    /* device status signals */
    private final StatusSignal<Angle> motorPosition = motor.getPosition(false);
    //private final StatusSignal<AngularVelocity> motorVelocity = motor.getVelocity(false);
    private final StatusSignal<Current> motorTorqueCurrent = motor.getTorqueCurrent(false);

    public HoodIOTalon() {
        for (int i = 0; i < HoodConstants.kNumConfigAttempts; ++i) {
            var status = motor.getConfigurator().apply(motorInitialConfigs);
            if (status.isOK())
                break;
        }

        BaseStatusSignal.setUpdateFrequencyForAll(20.0, motorPosition, motorTorqueCurrent);


        PhoenixUtil.registerSignals(false, motorPosition, motorTorqueCurrent);
    }

    


    public void updateInputs(HoodIOInputs inputs) {
        var refresh = PhoenixUtil.refreshAll();
        inputs.currentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.hoodAngle = motorPosition.getValueAsDouble();

        inputs.connected = refresh.isOK();
    }

    public void zero() {
        motor.setPosition(0.0);
    }

    public void setHoodAngle(Angle angle) {
        if (angle.baseUnitMagnitude() <= 90 && angle.baseUnitMagnitude() >= 0) {
            motor.setControl(m_ControlRequest.withPosition(angle));
        }
    }
    
}
