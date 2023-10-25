// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveUntilPitch extends CommandBase {
  /** Creates a new DriveUntilPitch. */
  private DriveSubsystem m_subsystem;
  private double m_x;
  private double m_y;
  private double m_pitch;
  public DriveUntilPitch(DriveSubsystem subsystem, double xSpeed, double ySpeed, double pitch) {
    m_subsystem = subsystem;
    m_x = xSpeed;
    m_y = ySpeed;
    m_pitch = pitch;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(m_x, m_y, 0, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.getRoll()) > m_pitch;
  }
}
