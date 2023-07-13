// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TestDriveCommand extends CommandBase {
    // Command to power driving motor

    private DriveSubsystem m_driveSubsystem;

    public TestDriveCommand(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(m_driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_driveSubsystem.drive4(1); 
    }

    @Override 
    public void execute() {
      SmartDashboard.putData(m_driveSubsystem);
    }

    // Called once the command ends or is interrupted.
  public void end() {
    m_driveSubsystem.drive4(0); 
  }
}
