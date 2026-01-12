package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drive.Drive;

import java.io.File;
import java.util.Arrays;
import java.util.LinkedList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command compAuto(Drive drive) {
    try {
      File[] choreoTrajs = new File("./src/main/deploy/choreo").listFiles((dir, name) -> name.endsWith(".traj"));
      Arrays.sort(choreoTrajs);

      SequentialCommandGroup mainAutoCommands = new SequentialCommandGroup();

      SequentialCommandGroup[] follows = new SequentialCommandGroup[choreoTrajs.length];
      for (int i = 0; i < choreoTrajs.length; i++) {
        follows[i] = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(choreoTrajs[i].getName().replaceAll("\\.traj", ""))).andThen(new WaitCommand(0.2));
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
