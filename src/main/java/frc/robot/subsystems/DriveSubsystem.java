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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(0);
  MaxSwerveModule m_frontRight = new MaxSwerveModule(Constants.k_FRDriveMotor, Constants.k_FRTurningMotor, 0);
  MaxSwerveModule m_frontLeft = new MaxSwerveModule(Constants.k_FLDriveMotor, Constants.k_FLTurningMotor, 0);
  MaxSwerveModule m_backRight = new MaxSwerveModule(Constants.k_BRDriveMotor, Constants.k_BRTurningMotor, 0);
  MaxSwerveModule m_backLeft = new MaxSwerveModule(Constants.k_BLDriveMotor, Constants.k_BLTurningMotor, 0);

  // Create the swerve chassis
  private final SwerveDriveKinematics m_driveKinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.k_driveLength / 2, Constants.k_driveWidth / 2), // front right
      new Translation2d(-Constants.k_driveLength / 2, Constants.k_driveWidth / 2), // front left
      new Translation2d(Constants.k_driveLength / 2, -Constants.k_driveWidth / 2), // back right
      new Translation2d(-Constants.k_driveLength / 2, -Constants.k_driveWidth / 2) // back left
    );

  // Slew rate limiter variables for lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.k_magnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.k_rotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Tracking robot position
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    m_driveKinematics, Rotation2d.fromDegrees(m_gyro.getAngle()), // ask about Constants.k_driveKinematics 
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    });

  public DriveSubsystem() {
    zeroHeading();
    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Angle In Ticks", m_backLeft.getAngle());
    m_odometry.update(
      Rotation2d.fromDegrees(m_gyro.getAngle()), 
      new SwerveModulePosition[] {
        m_frontRight.getPosition(), 
        m_frontLeft.getPosition(),
        m_backRight.getPosition(), 
        m_backLeft.getPosition(),
      }
    ); 
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters(); 
  }

  // Reset odometry to specified pose 
  public void resetOdometry(Pose2d pose) { 
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        },
        pose);
  }

  public void drive4(double s) {
    m_frontRight.drive1(s);
  }

  public void turn3(double s) {
    m_frontRight.turn1(s);
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
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
     
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
      SmartDashboard.putNumber("xSpeedCommanded", xSpeedCommanded);
      SmartDashboard.putNumber("ySpeedCommanded", ySpeedCommanded);
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

    var swerveModuleStates = m_driveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.k_maxSpeedMetersPerSecond);
    m_frontRight.setDesiredState(swerveModuleStates[0]);
    m_frontLeft.setDesiredState(swerveModuleStates[1]);
    m_backRight.setDesiredState(swerveModuleStates[2]);
    m_backLeft.setDesiredState(swerveModuleStates[3]);

    // System.out.println(String.format("xspeed=%f, yspeed=%f, rot=%f, fieldRelative=%b, rateLimit=%b", xSpeed, ySpeed, rot, fieldRelative, rateLimit)); 
  }

  // Accessor method for the chassis
  public SwerveDriveKinematics getDriveKinematics() {
    return m_driveKinematics;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.k_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  // Returns robot's heading in degrees from -180 to 180 
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  // Returns robot turn rate in degrees per second 
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.k_isGyroReversed ? -1.0 : 1.0);
  }
  
}
