// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands; 

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MaxSwerveModule;

public class SetPowerCommand extends CommandBase {
  /** Creates a new SetPowerCommand. */
  private DriveSubsystem m_driveSubsystem; 
  private double m_power; 

  public SetPowerCommand(DriveSubsystem driveSubsystem, double power) {
    m_driveSubsystem = driveSubsystem; 
    m_power = power; 
    addRequirements(m_driveSubsystem); 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.setPower(m_power); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
