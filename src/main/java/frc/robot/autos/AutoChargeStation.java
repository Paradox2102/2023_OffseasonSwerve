// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.ApriltagsCamera.Logger;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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

    // Commands
      new CreatePathCommand(m_subsystem, new Pose2d(4, 0, new Rotation2d(0)), List.of(new Translation2d(3, 0)), new Pose2d(2, 0, Rotation2d.fromDegrees(0)), true);
      addCommands(
      new InstantCommand(() -> Logger.log("AutoChargeStation", 0, "1")),
      new CreatePathCommand(m_subsystem, new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(2, 0)), new Pose2d(4, 0, new Rotation2d(0)), false),
      new InstantCommand(() -> Logger.log("AutoChargeStation", 0, "2")),
      new InstantCommand(() -> m_subsystem.drive(0, 0, 0, true, false)),
      new InstantCommand(() -> Logger.log("AutoChargeStation", 0, "3")),
      new WaitCommand(3),
      new InstantCommand(() -> Logger.log("AutoChargeStation", 0, "4")),
      // new CreatePathCommand(m_subsystem, new Pose2d(4, 0, new Rotation2d(0)), List.of(new Translation2d(3, 0)), new Pose2d(2, 0, Rotation2d.fromDegrees(0)), true),
      new InstantCommand(() -> Logger.log("AutoChargeStation", 0, "5"))
      // new AutoBalanceCommand(m_subsystem)
    );
  }
}
