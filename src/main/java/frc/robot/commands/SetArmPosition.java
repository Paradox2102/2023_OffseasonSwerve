// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmPosition extends InstantCommand {
  ArmPosition m_pose = ArmPosition.NEUTRAL;
  WristSubsystem m_wristSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  boolean m_neutralPose;
  public SetArmPosition(WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem, boolean neutralPose) {
    m_neutralPose = neutralPose;
    m_wristSubsystem = wristSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SetArmPosition initialize");
    double elevator = 0;
    double wrist = 0;
    m_pose = m_neutralPose ? ArmPosition.NEUTRAL : Constants.k_armPosition;
    switch (m_pose) {
      case NEUTRAL:
        elevator = Constants.k_neutralElevatorInches;
        wrist = Constants.k_neutralWristDegrees;
        break;
      case HIGH:
        wrist = Constants.k_isCubeMode ? Constants.CubeConstants.k_highWristDegrees : Constants.ConeConstants.k_highWristDegrees;
        elevator = Constants.k_isCubeMode ? Constants.CubeConstants.k_highElevatorInches : Constants.ConeConstants.k_highElevatorInches;
        break;
      case MID:
        wrist = Constants.k_isCubeMode ? Constants.CubeConstants.k_midWristDegrees : Constants.ConeConstants.k_midWristDegrees;
        elevator = Constants.k_isCubeMode ? Constants.CubeConstants.k_midElevatorInches : Constants.ConeConstants.k_midElevatorInches;
        break;
      case SINGLE:
        wrist = Constants.k_isCubeMode ? Constants.CubeConstants.k_singleWristDegrees : Constants.ConeConstants.k_singleWristDegrees;
        elevator = Constants.k_isCubeMode ? Constants.CubeConstants.k_singleElevatorInches : Constants.ConeConstants.k_singleElevatorInches;
        break;
      case DOUBLE:
        wrist = Constants.k_isCubeMode ? Constants.CubeConstants.k_doubleWristDegrees : Constants.ConeConstants.k_doubleWristDegrees;
        elevator = Constants.k_isCubeMode ? Constants.CubeConstants.k_doubleElevatorInches : Constants.ConeConstants.k_doubleElevatorInches;
        break;
      case GROUND:
        wrist = Constants.k_isCubeMode ? Constants.CubeConstants.k_groundWristDegrees : Constants.ConeConstants.k_groundWristDegrees;
        elevator = Constants.k_isCubeMode ? Constants.CubeConstants.k_groundElevatorInches : Constants.ConeConstants.k_groundElevatorInches;
        break;
    }
    m_wristSubsystem.setAngleDegrees(wrist);
    m_elevatorSubsystem.setExtentInches(elevator);
  }
}
