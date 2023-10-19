// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.DecideArmPosCommand;
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
  public AutoChargeStation(DriveSubsystem m_subsystem, WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // Commands
      addCommands(
        new TempIntakeCommand(intakeSubsystem, true),
        new SetGamePieceCommand(false),
        new DecideArmPosCommand(Constants.ArmPosition.HIGH),
        new SetArmPosition(wristSubsystem, elevatorSubsystem, false),
        new WaitCommand(1),
        new TempIntakeCommand(intakeSubsystem, false),
        new SetArmPosition(wristSubsystem, elevatorSubsystem, true),
        new WaitCommand(.75),
        new CreatePathCommand(m_subsystem, new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(2, 0)), new Pose2d(4, 0, new Rotation2d(0)), false, false),
        new AutoBalanceCommand(m_subsystem)
    );
  }
}
