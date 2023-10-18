// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.DecideArmPosCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetGamePieceCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceCube extends SequentialCommandGroup {
  /** Creates a new AutoPlaceCone. */
  public AutoPlaceCube(WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetGamePieceCommand(true),
      new DecideArmPosCommand(ArmPosition.HIGH),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, false),
      new WaitCommand(0.5),
      new OuttakeCommand(null),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, true),
      new WaitCommand(1)
    );
  }
}
