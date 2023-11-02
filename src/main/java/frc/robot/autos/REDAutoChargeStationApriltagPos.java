// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DecideArmPosCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetGamePieceCommand;
import frc.robot.commands.SetLEDColorCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class REDAutoChargeStationApriltagPos extends SequentialCommandGroup {
  /** Creates a new AutoMobility. 
   * @param wristSubsystem 
   * @param elevatorSubsystem 
   * @param intakeSubsystem 
   * @param ledSubsystem */
  public REDAutoChargeStationApriltagPos(DriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetGamePieceCommand(false),
      new DecideArmPosCommand(Constants.ArmPosition.HIGH),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, false),
      new WaitCommand(1),
      new ParallelRaceGroup(
        new IntakeCommand(intakeSubsystem, false),
        new WaitCommand(.25)
      ),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, true),
      new WaitCommand(.5),
      new SetLEDColorCommand(ledSubsystem, Color.kGreen, Color.kGreen),
      new DecideArmPosCommand(Constants.ArmPosition.GROUND),
      new SetArmPosition(wristSubsystem, elevatorSubsystem, false),
      new CreatePathCommand(driveSubsystem, new Pose2d(1.37795, 4.5, Rotation2d.fromDegrees(180)), List.of(new Translation2d(4.37795, 4.5)), new Pose2d(6, 4.5, Rotation2d.fromDegrees(180)), true, true),
      new WaitCommand(1.5),
      new CreatePathCommand(driveSubsystem, new Pose2d(6, 4.5, Rotation2d.fromDegrees(180)), List.of(new Translation2d(5, 4.5)), new Pose2d(4, 4.5, Rotation2d.fromDegrees(180)), false, false)
    );
  }
}
