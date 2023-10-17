// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.ApriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem; 

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CreatePathCommand extends SequentialCommandGroup {
  DriveSubsystem m_subsystem;
  private final double k_p = 1;
  private final double k_i = 0;
  private final double k_d = 0;

  public CreatePathCommand(DriveSubsystem driveSubsystem, Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, boolean isReversed, boolean resetPose) {
    m_subsystem = driveSubsystem;
    var thetaController = new ProfiledPIDController(
      1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    TrajectoryConfig config = new TrajectoryConfig(
      Constants.k_maxSpeedMetersPerSecond,
      Constants.k_maxDriveAcceleration)
      .setKinematics(m_subsystem.getSwerve());

    config.setReversed(isReversed);

    Trajectory path = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
  
    if (resetPose) {
      addCommands(
        new InstantCommand(() -> {m_subsystem.resetOdometry(path.getInitialPose()); Logger.log("ResetOdometry", 1, "initialize");})
      );   
    }
    addCommands(
      new SwerveControllerCommand(
      path,
      m_subsystem::getPose, // Functional interface to feed supplier
      m_subsystem.getSwerve(),

      // Position controllers
      new PIDController(k_p, k_i, k_d),
      new PIDController(k_p, k_i, k_d),
      thetaController,
      m_subsystem::setModuleStates,
      m_subsystem), 
      new RunCommand(() -> m_subsystem.setX(), driveSubsystem)
    );
  }

  public CreatePathCommand(DriveSubsystem driveSubsystem, Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, boolean isReversed, boolean resetPose, double maxSpeed, double maxAccel) {
    m_subsystem = driveSubsystem;
    TrajectoryConfig config = new TrajectoryConfig(
      maxSpeed,
      maxAccel)
      .setKinematics(m_subsystem.getSwerve());

    config.setReversed(isReversed);

      var thetaController = new ProfiledPIDController(
        1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
      Trajectory path = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);

      if (resetPose) {
        addCommands(
          new InstantCommand(() -> {m_subsystem.resetOdometry(path.getInitialPose());})
        );
      }
      addCommands(
        new SwerveControllerCommand(
        path,
        m_subsystem::getPose, // Functional interface to feed supplier
        m_subsystem.getSwerve(),
  
        // Position controllers
        new PIDController(k_p, k_i, k_d),
        new PIDController(k_p, k_i, k_d),
        thetaController,
        m_subsystem::setModuleStates,
        m_subsystem)
      );
  }

  public CreatePathCommand(DriveSubsystem driveSubsystem, Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, boolean isReversed, boolean resetPose, DoubleSupplier speed) {
    m_subsystem = driveSubsystem;
    TrajectoryConfig config = new TrajectoryConfig(
      Constants.k_maxSpeedMetersPerSecond,
      Constants.k_maxDriveAcceleration)
      .setKinematics(m_subsystem.getSwerve());

    // config.setReversed(isReversed);

    var thetaController = new ProfiledPIDController(
        1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    Trajectory path = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);

    if (resetPose) {
      addCommands(
        new InstantCommand(() -> {m_subsystem.resetOdometry(path.getInitialPose());})
      );
    }
    addCommands(
      new SwerveControllerCommand(
      path,
      m_subsystem::getPose, // Functional interface to feed supplier
      m_subsystem.getSwerve(),

      // Position controllers
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      thetaController,
      (SwerveModuleState[] states) -> m_subsystem.setModuleStatesWithSpeed(states, speed.getAsDouble()),
      m_subsystem).until(() -> false)
    );
  }
}
