// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2GamePieceBumpSide extends SequentialCommandGroup {
  /** Creates a new Auto2GamePieceBumpSide. */
  public Auto2GamePieceBumpSide(WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem, DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("2023_OffseasonSwerve");
    PathFollowingController controller = new PPHolonomicDriveController(new PIDConstants(1, 0, 0), new PIDConstants(1, 0, 0), 4.8, .6731);
    addCommands(
        new FollowPathCommand(paths.get(0), driveSubsystem::getPose, null, null, controller, null, driveSubsystem) 
    );
  }     
}