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
public class BLUEAuto2PieceNoBumpSide extends SequentialCommandGroup {
  /** Creates a new Auto2PieceNoBumpSide. */
  public BLUEAuto2PieceNoBumpSide(WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem,
      IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem) {
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
      new ParallelRaceGroup(
        new CreatePathCommand(
          driveSubsystem, 
          new Pose2d(1.8923, Constants.k_fieldWidthMeters - 4.9944274, Rotation2d.fromDegrees(180)), 
          List.of(new Translation2d(3.25, Constants.k_fieldWidthMeters - 5.2)),
          new Pose2d(7, Constants.k_fieldWidthMeters - 4.9, Rotation2d.fromDegrees(-1)), 
          true, 
          true
        ),
        new IntakeCommand(intakeSubsystem, true)
      ),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, true),

      new CreatePathCommand(
        driveSubsystem, 
        new Pose2d(7.05866, Constants.k_fieldWidthMeters - 4.9, Rotation2d.fromDegrees(-1)), 
        List.of(new Translation2d(4, Constants.k_fieldWidthMeters - 5.2)), 
        new Pose2d(1.89233, Constants.k_fieldWidthMeters - 5.3, Rotation2d.fromDegrees(-179)), 
        true, 
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
