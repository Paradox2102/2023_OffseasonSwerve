// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    Pose2d start = (new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
    Pose2d mid = (new Pose2d(1, 0, new Rotation2d(0)));
    Pose2d end = (new Pose2d(5, 0, Rotation2d.fromDegrees(180)));
    addCommands(
      // new CreatePathCommand(driveSubsystem, new Pose2d(1.8, -1, new Rotation2d(Math.PI)), List.of(new Translation2d(4, -.7)), new Pose2d(6.32, -.53, (new Rotation2d(Math.PI))), true, true)
      // new CreatePathCommand(driveSubsystem, new Pose2d(1.8, -1, (new Rotation2d(0))), List.of(new Translation2d(1.85, -1)), new Pose2d(1.9, -1, new Rotation2d(0)), true, true),
      new CreatePathCommand(driveSubsystem, start, List.of(mid.getTranslation()), end, true, true)
    );
  }
}
