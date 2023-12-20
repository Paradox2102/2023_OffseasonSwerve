// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class FollowPathCommand extends CommandBase {
  /** Creates a new FollowPathCommand. */
  DriveSubsystem m_subsystem;
  PathPlannerTrajectory m_path = new PathPlannerTrajectory();
  String m_pathFile;
  private double m_startTime = 0;
  private double m_trajStartTime = 0;
  public FollowPathCommand(DriveSubsystem driveSubsystem, String pathFile) {
    m_subsystem = driveSubsystem;
    m_pathFile = pathFile;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_path = PathPlanner.loadPath(m_pathFile, Constants.k_maxSpeedMetersPerSecond, Constants.k_maxDriveAcceleration);
    m_trajStartTime = (Timer.getFPGATimestamp() - m_startTime);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = (Timer.getFPGATimestamp() - m_startTime);
    // PathPlannerState currentState = (PathPlannerState)(m_path.sample(currentTime - m_trajStartTime));
    PathPlannerState nextState = (PathPlannerState)(m_path.sample(currentTime - m_trajStartTime + .02));

    Rotation2d nextHeading = nextState.holonomicRotation;

    m_subsystem.runHolonomicPath(nextState, nextHeading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
