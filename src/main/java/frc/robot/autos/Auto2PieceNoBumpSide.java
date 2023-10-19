// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.autos;

// import java.util.List;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Constants.ArmPosition;
// import frc.robot.commands.DecideArmPosCommand;
// import frc.robot.commands.IntakeCommand;
// import frc.robot.commands.OuttakeCommand;
// import frc.robot.commands.SetArmPosition;
// import frc.robot.commands.SetGamePieceCommand;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.WristSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class Auto2PieceNoBumpSide extends SequentialCommandGroup {
//   /** Creates a new Auto2PieceNoBumpSide. */
//   public Auto2PieceNoBumpSide(WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem,
//       IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem) {
//     Pose2d start = (new Pose2d(1.37795, -0.98806, Rotation2d.fromDegrees(180)));
//     Pose2d mid = (new Pose2d(5, -0.8, new Rotation2d(0)));
//     Pose2d gamePiece = (new Pose2d(7.058, -0.569, Rotation2d.fromDegrees(1)));
//     Pose2d end = (new Pose2d(1.37795, -0.55, Rotation2d.fromDegrees(180)));
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//         new SetGamePieceCommand(false),
//         new DecideArmPosCommand(ArmPosition.HIGH),
//         new SetArmPosition(wristSubsystem, elevatorSubsystem, false),
//         new WaitCommand(1),
//         new OuttakeCommand(intakeSubsystem),
//         new SetArmPosition(wristSubsystem, elevatorSubsystem, true),
//         new WaitCommand(0.5),
//         new SetGamePieceCommand(true),
//         new DecideArmPosCommand(ArmPosition.GROUND),
//         new SetArmPosition(wristSubsystem, elevatorSubsystem, false),

//         new CreatePathCommand(driveSubsystem, start, List.of(mid.getTranslation()), gamePiece, false, true),
//         new WaitCommand(0.5),
//         new IntakeCommand(intakeSubsystem),

//         new CreatePathCommand(driveSubsystem, gamePiece, List.of(mid.getTranslation()), end, false, false),
//         new DecideArmPosCommand(ArmPosition.HIGH),
//         new SetArmPosition(wristSubsystem, elevatorSubsystem, false),
//         new WaitCommand(1),
//         new OuttakeCommand(intakeSubsystem),
//         new SetArmPosition(wristSubsystem, elevatorSubsystem, true));
//   }
// }
