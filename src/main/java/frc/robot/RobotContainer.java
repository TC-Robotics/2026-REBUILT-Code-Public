package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalon;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOPhotonSim;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.subsystems.ExampleSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Subsystems
        private final Drive drive;
        private final Vision vision;
        private final Elevator elevator;
        private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

        // Controller
        private final CommandPS5Controller controller = new CommandPS5Controller(0);

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, IO devices, and commands.
         */
        public RobotContainer() {
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                                // a CANcoder
                                drive = new Drive(
                                                new GyroIOPigeon2(),
                                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                                new ModuleIOTalonFX(TunerConstants.BackRight));
                                vision = new Vision(
                                                drive::addVisionMeasurement,
                                                new VisionIOPhoton(camera0Name, robotToCamera0),
                                                new VisionIOPhoton(camera1Name, robotToCamera1));
                                
                                elevator = new Elevator(
                                        new ElevatorIOTalon()
                                );

                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIOSim(TunerConstants.FrontLeft),
                                                new ModuleIOSim(TunerConstants.FrontRight),
                                                new ModuleIOSim(TunerConstants.BackLeft),
                                                new ModuleIOSim(TunerConstants.BackRight));
                                // Sim robot, instantiate physics sim IO implementations
                                vision = new Vision(
                                                drive::addVisionMeasurement,
                                                new VisionIOPhotonSim(camera0Name, robotToCamera0,
                                                                drive::getPose),
                                                new VisionIOPhotonSim(camera1Name, robotToCamera1,
                                                                drive::getPose));
                                
                                elevator = new Elevator(
                                        new ElevatorIOSim()
                                );
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                });
                                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                                }, new VisionIO() {
                                });

                                elevator = new Elevator(
                                        new ElevatorIO() {}
                                );
                                break;
                }

                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

                // Set up SysId routines
                autoChooser.addOption(
                                "Drive Wheel Radius Characterization",
                                DriveCommands.wheelRadiusCharacterization(drive));
                autoChooser.addOption(
                                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Forward)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Reverse)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                autoChooser.addOption(
                        "Elevator Test: Ground", elevator.goToSetpoint(() -> Elevator.Setpoint.Ground));
                autoChooser.addOption(
                        "Elevator Test: Mid", elevator.goToSetpoint(() -> Elevator.Setpoint.MidScore));
                autoChooser.addOption(
                        "Elevator Test: High", elevator.goToSetpoint(() -> Elevator.Setpoint.HighScore));

                autoChooser.addDefaultOption("Competition Auto", Autos.compAuto(drive));

                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // Default command, normal field-relative drive
                drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> -controller.getLeftY(),
                                                () -> -controller.getLeftX(),
                                                () -> -controller.getRightX()));

                // Lock to 0° when triangle button is held
                controller
                                .triangle()
                                .whileTrue(
                                                DriveCommands.joystickDriveAtAngle(
                                                                drive,
                                                                () -> -controller.getLeftY(),
                                                                () -> -controller.getLeftX(),
                                                                () -> Rotation3d.kZero));

                // Switch to X pattern when cross button is pressed
                controller.cross().onTrue(Commands.runOnce(drive::stopWithX, drive));

                // Reset gyro to 0° when circle button is pressed
                controller
                                .circle()
                                .onTrue(
                                                Commands.runOnce(
                                                                () -> drive.setPose(
                                                                                new Pose3d(drive.getPose()
                                                                                                .getTranslation(),
                                                                                                Rotation3d.kZero)),
                                                                drive)
                                                                .ignoringDisable(true));

                controller.R2().onTrue(
                        elevator.goToSetpoint(() -> Elevator.Setpoint.HighScore)
                );

                controller.R1().onTrue(
                        elevator.goToSetpoint(() -> Elevator.Setpoint.MidScore)
                );

                controller.L2().onTrue(
                        elevator.goToSetpoint(() -> Elevator.Setpoint.Ground)
                );
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }
}