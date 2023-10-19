// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDrive extends CommandBase {
     /** Creates a new ArcadeDrive. */

  private DriveSubsystem m_subsystem;
  private DoubleSupplier m_getX;
  private DoubleSupplier m_getY;
  private DoubleSupplier m_getRot;
  private BooleanSupplier m_isFieldRelative;
  private BooleanSupplier m_isBalancing;

  public ArcadeDrive(DriveSubsystem driveSubsystem, DoubleSupplier getX, DoubleSupplier getY, DoubleSupplier getRot, BooleanSupplier isFieldRelative, BooleanSupplier isBalancing) {
    m_subsystem = driveSubsystem;
    m_getX = getX;
    m_getY = getY;
    m_getRot = getRot;
    m_isFieldRelative = isFieldRelative;
    m_isBalancing = isBalancing;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }
  //
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ArcadeDrive", 0, "initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_getX.getAsDouble();
    double y = m_getY.getAsDouble();
    double rot = m_getRot.getAsDouble();
    boolean isFieldRelative = m_isFieldRelative.getAsBoolean();
    boolean isBalancing = m_isBalancing.getAsBoolean();
    m_subsystem.drive(
      -MathUtil.applyDeadband(-y, Constants.k_driveDeadband), 
      -MathUtil.applyDeadband(-x, Constants.k_driveDeadband), 
      -MathUtil.applyDeadband(rot, Constants.k_driveDeadband), 
      isFieldRelative, 
      true
    );

    
    // m_swerve.setModuleStates(m_defaultState);
    // System.out.println(String.format("x=%f, y=%f, rot=%f, isFieldRelative=%b", x, y, rot, isFieldRelative)); 
  }
  
  

  // Called once the command ends or is interrupted.
  public void end() {
    m_subsystem.setModuleStates(Constants.k_defaultState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
