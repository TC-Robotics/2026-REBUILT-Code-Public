package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularAcceleration;

import edu.wpi.first.units.measure.Velocity;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.util.struct.StructGenerator;

public class ElevatorConstants {

    public static final CANBus kCANBus = new CANBus("rio");

    public static final Current kCurrentStatorLimit = Amps.of(120);

    public static final double kP = 32;
    public static final double kI = 0;
    public static final double kD = 0.4;

    public static final double kS = 0.2;
    public static final double kV = 0.96;
    public static final double kA = 0;

    public static final double kG = 0.5;

    public static final AngularVelocity MotionMagicCruiseVelocity = RotationsPerSecond.of(10);
    public static final AngularAcceleration MotionMagicAcceleration = RotationsPerSecondPerSecond.of(40);
    public static final Velocity<AngularAccelerationUnit> MotionMagicJerk = RotationsPerSecondPerSecond.per(Second).of(400);

    public static final Per<VoltageUnit, AngularVelocityUnit> Expo_kV = Volts.per(RotationsPerSecond).ofNative(0.96);
    public static final Per<VoltageUnit, AngularAccelerationUnit> Expo_kA = Volts.per(RotationsPerSecondPerSecond).ofNative(0.1);

    public static final int kNumConfigAttempts = 2;

    public static final double kGearRatio = 8;
    public static final Distance kDrumRadius = Meters.of(0.05);
    public static final Distance kMaxHeight = Meters.of(2);

    public static final double kSimLoopPeriod = 0.002; // 2 ms
}
