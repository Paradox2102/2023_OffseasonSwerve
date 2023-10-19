// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ParadoxField;
import frc.robot.PositionTrackerPose;
import frc.robot.subsystems.DriveSubsystem;

public class TempCreatePathCommand extends CommandBase {
  /** Creates a new TempCreatePathCommand. */
  DriveSubsystem m_subsystem;
  double k_distanceDeadZone;
  PositionTrackerPose m_tracker;
  double m_distance;
  Pose2d m_start;
  Pose2d m_end;
  double m_dx;
  double m_dy;

  double k_P = 1;
  double k_I = 0;
  double k_D = 0;
  PIDController m_xPID = new PIDController(k_P, k_I, k_D);
  PIDController m_yPID = new PIDController(k_P, k_I, k_D);

  double k_minPower = .05;

  public TempCreatePathCommand(DriveSubsystem driveSubsystem, Pose2d start, Pose2d end) {
    m_subsystem = driveSubsystem;
    m_tracker = m_subsystem.getTracker();
    m_start = start;
    m_end = end;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double m_dx = m_end.getX() - m_start.getX();
    double m_dy = -(m_end.getY() - m_start.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_dx = m_end.getX() - m_tracker.getPose2dFRC().getX();
    m_dx = (-m_end.getY() - m_tracker.getPose2dFRC().getY());
    m_distance = Math.sqrt((m_dx * m_dx) + (m_dy * m_dy));

    double heading = ParadoxField.rotation2dFromFRC(m_subsystem.getTracker().getPose2d().getRotation()).getDegrees();
    double rot = -(ParadoxField.normalizeAngle(heading - m_end.getRotation().getDegrees())) / 120.0;
    rot = Math.abs(rot) < k_minPower ? k_minPower * Math.signum(rot): rot;

    double xSpeed = m_xPID.calculate(m_dx);
    double ySpeed = m_xPID.calculate(m_dy);
    
    m_subsystem.drive(xSpeed, ySpeed, rot, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_distance) < k_distanceDeadZone;
  }
}
