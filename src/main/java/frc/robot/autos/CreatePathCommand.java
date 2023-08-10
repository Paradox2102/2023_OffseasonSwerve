// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CreatePathCommand extends SequentialCommandGroup {
  DriveSubsystem m_subsystem;

  public CreatePathCommand(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, boolean isReversed) {
    var thetaController = new ProfiledPIDController(
      1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    TrajectoryConfig config = new TrajectoryConfig(
      Constants.k_maxSpeedMetersPerSecond,
      Constants.k_maxDriveAcceleration)
      .setKinematics(m_subsystem.getSwerve());

    Trajectory path = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
    m_subsystem.resetOdometry(path.getInitialPose());
  
    addCommands(
      new SwerveControllerCommand(
      path,
      m_subsystem::getPose, // Functional interface to feed supplier
      m_subsystem.getSwerve(),

      // Position controllers
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      thetaController,
      m_subsystem::setModuleStates,
      m_subsystem)
    );
  }

  public CreatePathCommand(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, boolean isReversed, double maxSpeed, double maxAccel) {
    TrajectoryConfig config = new TrajectoryConfig(
      maxSpeed,
      maxAccel)
      .setKinematics(m_subsystem.getSwerve());

      var thetaController = new ProfiledPIDController(
        1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
      Trajectory path = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
      m_subsystem.resetOdometry(path.getInitialPose());
    
      addCommands(
        new SwerveControllerCommand(
        path,
        m_subsystem::getPose, // Functional interface to feed supplier
        m_subsystem.getSwerve(),
  
        // Position controllers
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        thetaController,
        m_subsystem::setModuleStates,
        m_subsystem)
      );
    
  }
}
