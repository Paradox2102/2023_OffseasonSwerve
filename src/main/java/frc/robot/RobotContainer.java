// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.ApriltagsCamera.ApriltagsCamera;
import frc.ApriltagsCamera.Logger;
import frc.robot.autos.AutoChargeStation;
import frc.robot.autos.CreatePathCommand;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoOrientCommand;
import frc.robot.commands.PartyMode;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetCoastModeCommand;
import frc.robot.commands.SetGamePieceCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.triggers.HoldTrigger;
import frc.robot.triggers.ToggleTrigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final ApriltagsCamera m_camera = new ApriltagsCamera(Constants.k_xFrontCameraOffsetInches, Constants.k_yFrontCameraOffsetInches, Constants.k_frontCameraAngle);
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_camera);
  public final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
  private final WristSubsystem m_wristSubsystem = new WristSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  public final PositionTrackerPose m_tracker = new PositionTrackerPose(0, 0, m_driveSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_xbox = new CommandXboxController(0);

  private AprilTagFieldLayout m_tags;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_driveSubsystem.setTracker(m_tracker);
    m_camera.connect("10.21.2.10", 5800);

    try {
      AprilTagFieldLayout tags = new AprilTagFieldLayout("/home/lvuser/deploy/2023-chargedup.json");
      m_tags = tags;
    } catch (IOException e) {
      Logger.log("RobotContainer", 1, "Field didn't load");
    }

    Trigger m_isFieldRelative = m_xbox.rightBumper();
    Trigger m_isBalancing = m_xbox.leftBumper();
    Trigger m_brakeMode = m_xbox.x();
    
    
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(
      m_driveSubsystem, 
      () -> m_xbox.getLeftX(), 
      () -> m_xbox.getLeftY(),
      () -> m_xbox.getRightX(),
      new ToggleTrigger(m_isFieldRelative.debounce(.1)),
      new HoldTrigger(m_isBalancing)
    ));
    m_xbox.leftBumper().whileTrue(new AutoBalanceCommand(m_driveSubsystem, () -> -m_xbox.getLeftY()));
    m_xbox.povDown().onTrue(new ResetGyro(m_driveSubsystem));
    m_xbox.b().onTrue(new SetGamePieceCommand(m_LEDSubsystem));
    m_xbox.x().onTrue(new SetCoastModeCommand(m_wristSubsystem, m_elevatorSubsystem, new ToggleTrigger(m_brakeMode.debounce(.1))));

    m_xbox.y().onTrue(new AutoOrientCommand(
      m_driveSubsystem, 
      0, 
      () -> -m_xbox.getLeftY(), 
      () -> m_xbox.getLeftX()
    ));

    m_xbox.a().onTrue(new AutoOrientCommand(
      m_driveSubsystem, 
      180, 
      () -> -m_xbox.getLeftY(), 
      () -> m_xbox.getLeftX()
    ));


    m_xbox.rightBumper().whileTrue(new CreatePathCommand(m_driveSubsystem, new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(1, 0)), new Pose2d(2, 0, new Rotation2d(0)), false, true, () -> -m_xbox.getLeftY()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() { 
    // System.out.println("Working"); 
    // System.out.print(String.format("x=%f, y=%f", m_xbox.getLeftX(), m_xbox.getLeftY()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutoChargeStation(m_driveSubsystem);
    // return new Auto344(m_driveSubsystem);
  }
}
