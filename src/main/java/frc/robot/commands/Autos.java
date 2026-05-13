package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

import java.io.File;
import java.util.Arrays;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;

/**
 * Factory methods for autonomous command sequences.
 */
public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  /**
   * Competition autonomous sequence that loads all Choreo paths in order and
   * follows them back-to-back with short delays for smoothing.
   */
  public static Command compAuto(Drive drive) {
    try {
      File[] choreoTrajs = new File("./src/main/deploy/choreo").listFiles((dir, name) -> name.endsWith(".traj"));
      Arrays.sort(choreoTrajs);

      // Build the full command sequence from all trajectory files.
      SequentialCommandGroup mainAutoCommands = new SequentialCommandGroup();

      Command[] follows = new Command[choreoTrajs.length];
      for (int i = 0; i < choreoTrajs.length; i++) {
        if (i == 0) {
          // First path is pathfound to the starting waypoint.
          follows[0] = AutoBuilder.pathfindThenFollowPath(
              PathPlannerPath.fromChoreoTrajectory(choreoTrajs[0].getName().replaceAll("\\.traj", "")),
              Constants.constraints)
              .andThen(new WaitCommand(0.2));
        }
        else {
          // Subsequent paths can be followed directly.
          follows[i] = AutoBuilder.followPath(
              PathPlannerPath.fromChoreoTrajectory(choreoTrajs[i].getName().replaceAll("\\.traj", "")))
              .andThen(new WaitCommand(0.2));
        }
      }

      mainAutoCommands.addCommands(follows);

      return mainAutoCommands;

    } catch (Exception e) {
      DriverStation.reportError("Getting the paths royally fucked up: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
