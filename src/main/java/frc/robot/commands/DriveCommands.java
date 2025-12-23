package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
    // drive constants
    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0;
    private static final double FF_RAMP_RATE = 0.1;
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25;
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05;

    private DriveCommands() {
    }

    // Converts joystick x/y inputs into a 3D pose (only 2D is actually used though)
    private static Translation3d getLinearVelocityFromJoysticks(double x, double y) {
        // deadband the controller input so it's not affected by stick drift
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);

        double yaw = Math.atan2(y, x);
        Rotation3d linearDirection = new Rotation3d(0.0, 0.0, yaw);

        linearMagnitude *= linearMagnitude; // Square the left joystick inputs

        // Creates a new Pose with no translation and the desired direction, then
        // transforms it and extracts the translation
        return new Pose3d(Translation3d.kZero, linearDirection)
                .transformBy(new Transform3d(linearMagnitude, 0.0, 0.0, Rotation3d.kZero))
                .getTranslation();
    }
    // For a PS5 controller, the xSupplier comes from the joystick rotation forwards and backwards (forwards is Negative Y / backwards is Positive Y)
    // the ySupplier comes from the joystick rotation left and right (left is Negative X / right is Positive X)
    // the omegaSupplier comes from the right joystick rotation left and right (left is Negative X / right is Positive X)
    // it's weird I know
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier, // Left joystick, backwards and forwards
            DoubleSupplier ySupplier, // Left joystick, left and right
            DoubleSupplier omegaSupplier) { // Right joystick, left and right

        return Commands.run(
                () -> {
                    Translation3d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                            ySupplier.getAsDouble());

                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
                    omega = Math.copySign(omega * omega, omega); // Square the right joystick input while preserving sign

                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega * drive.getMaxAngularSpeedRadPerSec());

                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;

                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI)).toRotation2d()
                                            : drive.getRotation().toRotation2d()));
                },
                drive);
    }

    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation3d> rotationSupplier) {

        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(
                () -> {
                    Translation3d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                            ySupplier.getAsDouble());

                    double omega = angleController.calculate(
                            drive.getRotation().getZ(),
                            rotationSupplier.get().getZ());

                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega);

                    boolean isFlipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;

                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI)).toRotation2d()
                                            : drive.getRotation().toRotation2d()));
                },
                drive)
                .beforeStarting(
                        () -> angleController.reset(drive.getRotation().getZ()));
    }

    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                        }),
                Commands.run(() -> drive.runCharacterization(0.0), drive)
                        .withTimeout(FF_START_DELAY),
                Commands.runOnce(timer::restart),
                Commands.run(
                        () -> {
                            double voltage = timer.get() * FF_RAMP_RATE;
                            drive.runCharacterization(voltage);
                            velocitySamples.add(drive.getFFCharacterizationVelocity());
                            voltageSamples.add(voltage);
                        },
                        drive)
                        .finallyDo(
                                () -> {
                                    int n = velocitySamples.size();
                                    double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println("********** Drive FF Characterization Results **********");
                                    System.out.println("\tkS: " + formatter.format(kS));
                                    System.out.println("\tkV: " + formatter.format(kV));
                                }));
    }

    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                Commands.sequence(
                        Commands.runOnce(() -> limiter.reset(0.0)),
                        Commands.run(
                                () -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0,
                                        limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY))),
                                drive)),
                Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(
                                () -> {
                                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = drive.getRotation();
                                    state.gyroDelta = 0.0;
                                }),
                        Commands.run(
                                () -> {
                                    Rotation3d rotation = drive.getRotation();
                                    state.gyroDelta += Math.abs(
                                            rotation.minus(state.lastAngle).getZ());
                                    state.lastAngle = rotation;
                                })
                                .finallyDo(
                                        () -> {
                                            double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                            double wheelDelta = 0.0;
                                            for (int i = 0; i < 4; i++) {
                                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                            }
                                            double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS)
                                                    / wheelDelta;

                                            NumberFormat formatter = new DecimalFormat("#0.000");
                                            System.out.println(
                                                    "********** Wheel Radius Characterization Results **********");
                                            System.out.println(
                                                    "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                            System.out.println(
                                                    "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                            System.out.println(
                                                    "\tWheel Radius: "
                                                            + formatter.format(wheelRadius)
                                                            + " meters, "
                                                            + formatter.format(Units.metersToInches(wheelRadius))
                                                            + " inches");
                                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation3d lastAngle = Rotation3d.kZero;
        double gyroDelta = 0.0;
    }
}
