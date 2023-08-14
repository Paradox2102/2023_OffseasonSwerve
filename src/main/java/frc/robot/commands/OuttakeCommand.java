// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeType;
import frc.robot.subsystems.LEDSubsystem.LEDMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// OUTTAKES AND SETS ARM BACK TO NEUTRAL POSITION

public class OuttakeCommand extends SequentialCommandGroup {
  /** Creates a new OuttakeCommand. */
  public OuttakeCommand(IntakeSubsystem intakeSubsystem, WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem, LEDSubsystem ledSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeCommand(intakeSubsystem, IntakeType.OUTTAKE),
      new WaitCommand(.5),
      new IntakeCommand(intakeSubsystem, IntakeType.STOP),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, Constants.ArmPosition.NEUTRAL),
      new SignalLEDCommand(ledSubsystem, LEDMode.READY)
    );
  }
}
