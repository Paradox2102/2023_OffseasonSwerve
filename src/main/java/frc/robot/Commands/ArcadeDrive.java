// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MaxSwerveModule;

public class ArcadeDrive extends CommandBase {
     /** Creates a new ArcadeDrive. */

  private MaxSwerveModule m_swerveModule;
  private XboxController m_xbox; 

  private final boolean k_fieldRelative = true; 
  private final static double k_deadZone = 0.02; 

  public ArcadeDrive(XboxController xbox, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_xbox = xbox;
    // m_swerveModule = driveSubsystem.getSwerve();
    addRequirements(driveSubsystem);
  }
  
  private void driveWithXbox(boolean fieldRelative) { /* 
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_xbox.getLeftY(), k_deadZone))
    * MaxSwerveModule.kMaxSpeed;

    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_xbox.getLeftX(), k_deadZone))
    * Drivetrain.kMaxSpeed;

    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_xbox.getRightX(), k_deadZone))
    * Drivetrain.kMaxAngularSpeed;

    // System.out.println(String.format("xspeed=%f, yspeed=%f, rot=%f, fieldRelative=%b", xSpeed, ySpeed, rot, fieldRelative)); 

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    */
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_xbox != null){   
      driveWithXbox(k_fieldRelative);
    }
    else {
      driveWithXbox(k_fieldRelative); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_swerveModule.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
