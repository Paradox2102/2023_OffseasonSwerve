// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoChargeStationTemp extends CommandBase {
  /** Creates a new AutoChargeStationTemp. */
  DriveSubsystem m_subsystem;
  private final double k_jerkPower = -.1;
  private final double k_jerTime = .25;
  double m_previousPitch = 0;
  Timer m_jerkTimer = new Timer();

  public AutoChargeStationTemp(DriveSubsystem driveSubsystem) {
    m_subsystem = driveSubsystem;
    m_jerkTimer.reset();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_jerkTimer.reset();
    m_subsystem.drive(.025, 0, 0, true, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPitch = m_subsystem.getRoll();
    if (currentPitch < m_previousPitch) {
      m_jerkTimer.start();
      m_subsystem.drive(k_jerkPower, 0, 0, true, false);
    }
    m_previousPitch = currentPitch;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setX();
    m_jerkTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_jerkTimer.get() > k_jerTime;
  }
}
