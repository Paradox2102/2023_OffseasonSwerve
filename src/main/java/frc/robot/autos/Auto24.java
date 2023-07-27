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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto24 extends SequentialCommandGroup {
  /** Creates a new Auto24. */
  public Auto24(DriveSubsystem m_subsystem) {
    SwerveDriveKinematics swerve = m_subsystem.getSwerve();
    TrajectoryConfig config = new TrajectoryConfig(1, 2).setKinematics(swerve);
    
    Trajectory path = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(0, 1)),// new Translation2d(1, 1), new Translation2d(0, 1)),
      new Pose2d(0, 2, new Rotation2d(0)),
      config
    );
    var thetaController = new ProfiledPIDController(
        1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_subsystem.resetOdometry(path.getInitialPose());
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SwerveControllerCommand(
        path, 
        m_subsystem::getPose, 
        swerve, 
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        thetaController,
        m_subsystem::setModuleStates, 
        m_subsystem
      )
    );

  }
}
