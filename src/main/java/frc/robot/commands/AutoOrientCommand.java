// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ParadoxField;
import frc.robot.subsystems.DriveSubsystem;

public class AutoOrientCommand extends CommandBase {
  /** Creates a new AutoOrientCommand. */
  DriveSubsystem m_subsystem;
  DoubleSupplier m_y;
  DoubleSupplier m_x;
  double m_angle;
  double k_deadzone = 2.5;
  double k_minPower = .2;
  public AutoOrientCommand(DriveSubsystem driveSubsystem, double angle, DoubleSupplier y, DoubleSupplier x) {
    m_subsystem = driveSubsystem;
    m_y = y;
    m_x = x;
    m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_x.getAsDouble();
    double y = m_y.getAsDouble();
    double heading = m_subsystem.getHeadingInDegrees();
    double rot = -(ParadoxField.normalizeAngle(heading - m_angle)) / 120.0;
    rot = Math.abs(rot) < k_minPower ? k_minPower * Math.signum(rot): rot;
    m_subsystem.drive(-y, x, rot, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.getHeadingInDegrees() - m_angle) <= k_deadzone;
  }
}
