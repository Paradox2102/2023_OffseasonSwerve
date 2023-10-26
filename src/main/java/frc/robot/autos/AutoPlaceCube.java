// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.DecideArmPosCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetGamePieceCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceCube extends SequentialCommandGroup {
  /** Creates a new AutoPlaceCone. */
  public AutoPlaceCube(DriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetGyro(driveSubsystem, 180),
      new SetGamePieceCommand(true),
      new DecideArmPosCommand(ArmPosition.HIGH),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, false),
      new WaitCommand(2),
      new ParallelRaceGroup(
        new IntakeCommand(intakeSubsystem, false),
        new WaitCommand(1)
      ),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, true)
    );
  }
}
