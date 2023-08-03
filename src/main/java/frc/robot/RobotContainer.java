// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.Auto24;
import frc.robot.autos.AutoChargeStation;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_xbox = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    Trigger m_isFieldRelative = m_xbox.rightBumper();
    
    
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(
      m_driveSubsystem, 
      () -> m_xbox.getLeftX(), 
      () -> m_xbox.getLeftY(),
      () -> m_xbox.getRightX(),
      new ToggleTrigger(m_isFieldRelative.debounce(.1))
    ));
    m_xbox.leftBumper().whileTrue(new AutoBalanceCommand(m_driveSubsystem));
      // System.out.print(String.format("x=%f, y=%f", m_xbox.getLeftX(), m_xbox.getLeftY()));
      // System.out.println("Working"); 

    // m_driveSubsystem.setDefaultCommand(
    //   // Left stick controls translation of robot 
    //   // Right stick's x-axis controls turning 
    //   new RunCommand(
    //     () -> m_driveSubsystem.drive(
    //       -MathUtil.applyDeadband(m_xbox.getLeftY(), Constants.k_driveDeadband), 
    //       -MathUtil.applyDeadband(m_xbox.getLeftX(), Constants.k_driveDeadband), 
    //       -MathUtil.applyDeadband(m_xbox.getRightX(), Constants.k_driveDeadband),
    //       true, true), 
    //       m_driveSubsystem)); 
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
  }
}
