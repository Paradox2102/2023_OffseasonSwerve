// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPlannerTester extends SequentialCommandGroup {
  /** Creates a new PathPlannerTester. */
  public PathPlannerTester(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, WristSubsystem wristSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // ArrayList<PathPlannerTrajectory> paths = PathPlanner
    PathPlannerTrajectory paths1 = PathPlanner.loadPath("Bump1", Constants.k_maxSpeedMetersPerSecond, Constants.k_maxDriveAcceleration);
    PathPlannerTrajectory path2 = PathPlanner.loadPath("Bump2", Constants.k_maxSpeedMetersPerSecond, Constants.k_maxDriveAcceleration);
    addCommands(
      new ScoreConeHigh(elevatorSubsystem, intakeSubsystem, wristSubsystem)
    );
  }
}
