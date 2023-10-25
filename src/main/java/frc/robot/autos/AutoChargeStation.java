// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoOrientCommand;
import frc.robot.commands.DecideArmPosCommand;
import frc.robot.commands.ResetGyro;
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
public class AutoChargeStation extends SequentialCommandGroup {
  /** Creates a new AutoChargeStation. */
  public AutoChargeStation(DriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Commands
      addCommands(
        new ResetGyro(driveSubsystem, 180),
        new SetGamePieceCommand(false),
        new TempIntakeCommand(intakeSubsystem, true),
        new DecideArmPosCommand(Constants.ArmPosition.HIGH),
        new SetArmPosition(wristSubsystem, elevatorSubsystem, false),
        new WaitCommand(1),
        new TempIntakeCommand(intakeSubsystem, false),
        new SetArmPosition(wristSubsystem, elevatorSubsystem, true),
        new WaitCommand(.75),
        new ParallelRaceGroup(
          new RunCommand(() -> driveSubsystem.drive(-2.5, 0, 0, true, false)),
          new WaitCommand(.1)
        ),
        new AutoOrientCommand(driveSubsystem, 0, () -> 0, () -> 0),
        new DecideArmPosCommand(Constants.ArmPosition.GROUND),
        new SetArmPosition(wristSubsystem, elevatorSubsystem, false),
        new AutoBalanceCommand(driveSubsystem)
    );
  }
}
