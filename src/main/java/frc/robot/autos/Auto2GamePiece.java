// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2GamePiece extends SequentialCommandGroup {
  /** Creates a new Auto2GamePiece. */
  public Auto2GamePiece(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new CreatePathCommand(driveSubsystem, new Pose2d(1, 2.52, new Rotation2d(Math.PI)), List.of(new Translation2d(2.5, 2.6)), new Pose2d(5.7, 2.7, (new Rotation2d(.1))), true, true));
  }
}
