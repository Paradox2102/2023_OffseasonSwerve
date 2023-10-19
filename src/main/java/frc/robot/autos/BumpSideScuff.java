// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DecideArmPosCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetGamePieceCommand;
import frc.robot.commands.TempIntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BumpSideScuff extends SequentialCommandGroup {
  /** Creates a new BumpSideScuff. */
  public BumpSideScuff(DriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetGamePieceCommand(false),
      new DecideArmPosCommand(Constants.ArmPosition.HIGH),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, false),
      new WaitCommand(1),
      new ParallelRaceGroup(new WaitCommand(.5), new TempIntakeCommand(intakeSubsystem, false)),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, true),
      new ParallelRaceGroup(new WaitCommand(5), new RunCommand(() -> driveSubsystem.drive(.25, 0, 0, true, false), driveSubsystem))
    );
  }
}
