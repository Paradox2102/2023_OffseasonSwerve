// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoChargeStation extends SequentialCommandGroup {
  /** Creates a new AutoChargeStation. */
  public AutoChargeStation(DriveSubsystem m_subsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    TrajectoryConfig configF = new TrajectoryConfig(
        1,
        Constants.k_maxDriveAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(m_subsystem.getSwerve());

      TrajectoryConfig configB = new TrajectoryConfig(
        1,
        Constants.k_maxDriveAcceleration
      )
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(m_subsystem.getSwerve());

      configB.setReversed(true);


    Trajectory path1 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(2, 0)),
      new Pose2d(4, 0, Rotation2d.fromDegrees(0)),
      configF);

    Trajectory path2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(4, 0, new Rotation2d(0)),
      List.of(new Translation2d(3, 0)),
      new Pose2d(
        2, 0, Rotation2d.fromDegrees(0)),
      configB);

    var thetaController = new ProfiledPIDController(
        1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_subsystem.resetOdometry(path1.getInitialPose());

    // Commands
    SwerveControllerCommand command1 = new SwerveControllerCommand(
      path1,
      m_subsystem::getPose, // Functional interface to feed supplier
      m_subsystem.getSwerve(),

      // Position controllers
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      thetaController,
      m_subsystem::setModuleStates,
      m_subsystem);

    SwerveControllerCommand command2 = new SwerveControllerCommand(
      path2,
      m_subsystem::getPose, // Functional interface to feed supplier
      m_subsystem.getSwerve(),

      // Position controllers
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      thetaController,
      m_subsystem::setModuleStates,
      m_subsystem);
    

    addCommands(
      command1, 
      command2,
      new AutoBalanceCommand(m_subsystem)
    );
  }
}
