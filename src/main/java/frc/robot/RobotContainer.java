// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.ApriltagsCamera.ApriltagsCamera    ;
import frc.ApriltagsCamera.Logger;
import frc.robot.Constants.ArmPosition;
import frc.robot.autos.Auto2GamePiece;
import frc.robot.autos.Auto2GamePieceBumpSide;
import frc.robot.autos.AutoChargeStation;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoOrientCommand;
import frc.robot.commands.DecideArmPosCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ResetWrist;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetCoastModeCommand;
import frc.robot.commands.SetLEDColorCommand;
import frc.robot.commands.manual.ManualElevatorCommand;
import frc.robot.commands.manual.ManualWristCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
  private final WristSubsystem m_wristSubsystem = new WristSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  public final PositionTrackerPose m_tracker = new PositionTrackerPose(0, 0, m_driveSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_xbox = new CommandXboxController(0);
  private final CommandJoystick m_stick = new CommandJoystick(1); 

  private AprilTagFieldLayout m_tags;

  SendableChooser<Command> m_selectAuto = new SendableChooser<>();

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
  }

  public boolean getThrottle() {
    return m_stick.getThrottle() < 0;
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
    Trigger m_isFieldRelative = m_xbox.rightBumper();
    Trigger m_isBalancing = m_xbox.leftBumper();
    Trigger m_brakeMode = m_xbox.x();

    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(
      m_driveSubsystem, 
      () -> m_xbox.getLeftX(), 
      () -> m_xbox.getLeftY(),
      () -> m_xbox.getRightX(),
      new HoldTrigger(m_isFieldRelative),
      new HoldTrigger(m_isBalancing)
    ));

    // Driver 1
    // m_xbox.leftBumper().whileTrue(new AutoBalanceCommand(m_driveSubsystem, () -> -m_xbox.getLeftY()));
    m_xbox.povUp().onTrue(new ResetGyro(m_driveSubsystem, 0));
    m_xbox.povRight().onTrue(new ResetGyro(m_driveSubsystem, 90));
    m_xbox.povDown().onTrue(new ResetGyro(m_driveSubsystem, 180));
    m_xbox.povDown().onTrue(new ResetGyro(m_driveSubsystem, 270));

    m_xbox.a().onTrue(new AutoOrientCommand(
      m_driveSubsystem, 
      180, 
      () -> -m_xbox.getLeftY(), 
      () -> m_xbox.getLeftX()
    ));
    m_xbox.b().onTrue(new AutoOrientCommand(
      m_driveSubsystem, 
      90, 
      () -> -m_xbox.getLeftY(), 
      () -> m_xbox.getLeftX()
    ));

    m_xbox.x().onTrue(new AutoOrientCommand(
      m_driveSubsystem, 
      270, 
      () -> -m_xbox.getLeftY(), 
      () -> m_xbox.getLeftX()
    ));

    m_xbox.y().onTrue(new AutoOrientCommand(
      m_driveSubsystem, 
      0, 
      () -> -m_xbox.getLeftY(), 
      () -> m_xbox.getLeftX()
    ));

    m_xbox.rightTrigger().whileTrue(new IntakeCommand(m_intakeSubsystem));
    m_xbox.leftTrigger().onTrue(new OuttakeCommand(m_intakeSubsystem));
    m_xbox.rightBumper().onTrue(new SetArmPosition(m_wristSubsystem, m_elevatorSubsystem, false));
    m_xbox.leftBumper().onTrue(new SetArmPosition(m_wristSubsystem, m_elevatorSubsystem, true));

    // Driver 2
    m_stick.button(4).whileTrue(new ManualWristCommand(m_wristSubsystem, () -> m_stick.getY()));
    m_stick.button(6).whileTrue(new ManualElevatorCommand(m_elevatorSubsystem, () -> m_stick.getY()));
    
    m_stick.button(5).onTrue(new ResetWrist(m_wristSubsystem));
    m_stick.button(8).onTrue(new SetCoastModeCommand(m_wristSubsystem, m_elevatorSubsystem, new ToggleTrigger(m_brakeMode)));

    m_stick.button(7).onTrue(new DecideArmPosCommand(ArmPosition.HIGH));
    m_stick.button(9).onTrue(new DecideArmPosCommand(ArmPosition.MID));
    m_stick.button(11).onTrue(new DecideArmPosCommand(ArmPosition.GROUND));
    m_stick.button(10).onTrue(new DecideArmPosCommand(ArmPosition.DOUBLE));
    m_stick.button(12).onTrue(new SetLEDColorCommand(m_ledSubsystem, Color.kRed, Color.kGreen));

    m_stick.button(1).toggleOnTrue(new RunCommand(() -> m_driveSubsystem.setOnePower()));
    m_stick.button(2).toggleOnTrue(new RunCommand(() -> m_intakeSubsystem.setPower(.5)));

    // Auto Selection
    m_selectAuto.addOption("Test Auto", testAuto2GamePiece = new Auto2GamePiece(m_driveSubsystem));
    m_selectAuto.addOption("Test Balance", testAutoBalance = new AutoBalanceCommand(m_driveSubsystem));
    m_selectAuto.addOption("Charge Station", autoChargeStation = new AutoChargeStation(m_driveSubsystem));
    m_selectAuto.addOption("No Bump 2", auto2PieceNoBump = new Auto2GamePieceBumpSide(m_wristSubsystem, m_elevatorSubsystem, m_driveSubsystem, m_intakeSubsystem));
    m_selectAuto.addOption("Bump 2", auto2PieceBump = new Auto2GamePieceBumpSide(m_wristSubsystem, m_elevatorSubsystem, m_driveSubsystem, m_intakeSubsystem));

    new Trigger(() -> Constants.k_isCubeMode).onTrue(new SetLEDColorCommand(m_ledSubsystem, Color.kPurple, Color.kPurple));
    new Trigger(() -> Constants.k_isCubeMode).onFalse(new SetLEDColorCommand(m_ledSubsystem, Color.kOrange, Color.kOrange));

    SmartDashboard.putData(m_selectAuto);
  }
  Command testAuto2GamePiece;
  Command testAutoBalance;
  Command autoChargeStation;
  Command auto2PieceNoBump;
  Command auto2PieceBump;

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_selectAuto.getSelected();
  }
}
