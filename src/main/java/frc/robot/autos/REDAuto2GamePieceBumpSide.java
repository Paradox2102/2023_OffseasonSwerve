// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DecideArmPosCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetGamePieceCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class REDAuto2GamePieceBumpSide extends SequentialCommandGroup {
  /** Creates a new Auto2GamePieceBumpSide. */
  public REDAuto2GamePieceBumpSide(WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem, DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetGamePieceCommand(false),
      new DecideArmPosCommand(Constants.ArmPosition.HIGH),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, false),
      new WaitCommand(1),
      new ParallelRaceGroup(
        new IntakeCommand(intakeSubsystem, false),
        new WaitCommand(.5)
      ),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, true),
      new WaitCommand(.75),
      new SetGamePieceCommand(true),
      new DecideArmPosCommand(Constants.ArmPosition.GROUND),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, false),
      
      new CreatePathCommand(
        driveSubsystem, 
        new Pose2d(1.37795, 7.2471284, Rotation2d.fromDegrees(180)), 
        List.of(new Translation2d(2, 3.2), new Translation2d(6, 3.8)), 
        new Pose2d(7.068, 7.1471284, Rotation2d.fromDegrees(1)), 
        true, 
        true
      ),
      // new SetArmPosition(wristSubsystem, elevatorSubsystem, true),

      new CreatePathCommand(
        driveSubsystem, 
        new Pose2d(7.068, 3.09, Rotation2d.fromDegrees(1)), 
        List.of(new Translation2d(6, 3.8), new Translation2d(2, 3.2)), 
        new Pose2d(1.37795, 7.0471284, Rotation2d.fromDegrees(180)), 
        false, 
        false
      ),
      new InstantCommand(() -> driveSubsystem.setX(), driveSubsystem),
      
      new DecideArmPosCommand(Constants.ArmPosition.HIGH),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, false),
      new WaitCommand(1),
      new ParallelRaceGroup(
        new IntakeCommand(intakeSubsystem, false),
        new WaitCommand(.5)
      ),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, true)
      
    );
  }
}