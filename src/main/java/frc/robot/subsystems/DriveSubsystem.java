// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.ApriltagsCamera.ApriltagsCamera;
import frc.robot.Constants;
import frc.robot.ParadoxField;
import frc.robot.PositionTrackerPose;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final Field2d m_field = new Field2d();
  // Create MaxSwerveModules
  private final MaxSwerveModule m_frontLeft = new MaxSwerveModule(
      Constants.k_FLDriveMotor,
      Constants.k_FLTurningMotor,
      Constants.k_FLOffset);

  private final MaxSwerveModule m_frontRight = new MaxSwerveModule(
      Constants.k_FRDriveMotor,
      Constants.k_FRTurningMotor,
      Constants.k_FROffset);

  private final MaxSwerveModule m_backLeft = new MaxSwerveModule(
      Constants.k_BLDriveMotor,
      Constants.k_BLTurningMotor,
      Constants.k_BLOffset);

  private final MaxSwerveModule m_backRight = new MaxSwerveModule(
      Constants.k_BRDriveMotor,
      Constants.k_BRTurningMotor,
      Constants.k_BROffset);
      

  // The gyro sensor
  private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(0);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.k_magnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.k_rotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private final SwerveDriveKinematics m_swerve = new SwerveDriveKinematics(
        new Translation2d(.33655, .33655),
        new Translation2d(.33655, -.33655),
        new Translation2d(-.33655, .33655),
        new Translation2d(-.33655, -.33655));

    PositionTrackerPose m_tracker = null;
    ApriltagsCamera m_camera;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(ApriltagsCamera camera) {
    m_gyro.reset();
    SmartDashboard.putData("Field", m_field);

    m_camera = camera;
  }

  public void setPower() {
    drive(1, 0, 0, true, false);
  }

  public SwerveDriveKinematics getSwerve() {
    return m_swerve;
  }

  public double getPitch() {
    return m_gyro.getPitch();
  }

  public void setTracker(PositionTrackerPose tracker) {
    m_tracker = tracker;
  }



  public SwerveModulePosition[] getModulePosition() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  public double getRoll() {
    return m_gyro.getRoll();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    SmartDashboard.putNumber("ATurn FR", (m_frontRight.getAngle()));///Math.PI);
    SmartDashboard.putNumber("ATurn FL", m_frontLeft.getAngle());// - (Math.PI / 2)) / Math.PI);
    SmartDashboard.putNumber("ATurn BR", m_backRight.getAngle());// + (Math.PI / 2)) / Math.PI);
    SmartDashboard.putNumber("ATurn BL", m_backLeft.getAngle());// + (Math.PI)) / Math.PI);
    // SmartDashboard.putData("Gyro Angle", m_gyro);
    // SmartDashboard.putNumber("Roll", getRoll());
    // SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Pose Est X", (m_tracker.getPose2dFRC().getTranslation().getX()));
    SmartDashboard.putNumber("Pose Est Y", (m_tracker.getPose2dFRC().getTranslation().getY()));
    SmartDashboard.putNumber("Pose Est Rot", (m_tracker.getPose2dFRC().getRotation().getDegrees()));
    System.out.println(m_tracker.getPose2dFRC().getRotation().getDegrees());

    m_tracker.update(m_camera);
    m_field.setRobotPose(m_tracker.getPose2dFRC().getTranslation().getX(), m_tracker.getPose2dFRC().getTranslation().getY(), m_tracker.getPose2dFRC().getRotation());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_tracker.getPose2dFRC();
  }

  public Field2d getField() {
    return m_field;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    double angle = pose.getRotation().getDegrees();
    m_gyro.setYaw(angle);
    m_tracker.setXYAngleFRC(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    System.out.println(pose.getX()+" "+ pose.getY()+ " "+pose.getRotation().getDegrees());
    System.out.println(m_tracker.getPose2dFRC().getTranslation());
}

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(Constants.k_directionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * Constants.k_maxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * Constants.k_maxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * Constants.k_maxAngularSpeed;

    var swerveModuleStates = m_swerve.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, ParadoxField.rotation2dFromFRC(m_tracker.getPose2d().getRotation()))
            : new ChassisSpeeds(-xSpeedDelivered, -ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.k_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public Rotation2d getGyroRotation2d() {
    return m_gyro.getRotation2d();
  }

  public WPI_PigeonIMU getGyro() {
    return m_gyro;
  }

  public PositionTrackerPose getTracker() {
    return m_tracker;
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.k_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public void setModuleStatesWithSpeed(SwerveModuleState[] desiredStates, double speed) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.k_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredStateWithSpeed(desiredStates[0], speed);
    m_frontRight.setDesiredStateWithSpeed(desiredStates[1], speed);
    m_backLeft.setDesiredStateWithSpeed(desiredStates[2], speed);
    m_backRight.setDesiredStateWithSpeed(desiredStates[3], speed);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }
 
  public void setOnePower() {
    m_backLeft.setDrive();
  }

  /** Zeroes the heading of the robot. */
  public void setHeading(double angle) {
    m_tracker.setXYAngleFRC(m_tracker.getPose2dFRC().getX(), m_tracker.getPose2dFRC().getY(), angle);
    System.out.println("yes");
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    double angle =  -Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
    angle %= 360;
    angle = (angle + 360) % 360;
    if (angle > 180) {
      angle -= 360;
    }
    return angle;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.k_gyroReversed ? -1.0 : 1.0);
  }
}