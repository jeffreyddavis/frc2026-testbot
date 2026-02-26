// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.opencv.features2d.FlannBasedMatcher;

//import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FlipUtil;
import frc.robot.RobotContainer;
import gg.questnav.questnav.QuestNav;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.Matrix;


import edu.wpi.first.wpilibj.Timer;


/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 4.4196; // 3.0 3 meters per second
  public static final double kMaxAngularSpeedSet = Math.PI*1;
  public static final double kMaxAngularSpeedFastSet = Math.PI*4;
  public static double kMaxAngularSpeed = Math.PI * 1; // (without /3) 1/2 rotation per second

  //.3048 meters = 12 inches
  // .432 meters = 17 inches
  private final Translation2d m_frontLeftLocation = new Translation2d(0.432, 0.432);
  private final Translation2d m_frontRightLocation = new Translation2d(0.432, -0.432);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.432, 0.432);
  private final Translation2d m_backRightLocation = new Translation2d(-0.432, -0.432);

  private final SwerveModule m_frontLeft = new SwerveModule(
    Constants.SwerveModuleIds.FrontLeftDrive,
    Constants.SwerveModuleIds.FrontLeftTurn,
    Constants.SwerveModuleIds.FrontLeftEncoder, false);

  private final SwerveModule m_frontRight = new SwerveModule(
    Constants.SwerveModuleIds.FrontRightDrive,
    Constants.SwerveModuleIds.FrontRightTurn,
    Constants.SwerveModuleIds.FrontRightEncoder, true);
    
public final QuestNav questNav = new QuestNav();

  private final SwerveModule m_backLeft =  new SwerveModule(
    Constants.SwerveModuleIds.BackLeftDrive,
    Constants.SwerveModuleIds.BackLeftTurn,
    Constants.SwerveModuleIds.BackLeftEncoder, false);
  
  
  private final SwerveModule  m_backRight = new SwerveModule(
    Constants.SwerveModuleIds.BackRightDrive,
    Constants.SwerveModuleIds.BackRightTurn,
    Constants.SwerveModuleIds.BackRightEncoder, true);

  public Pigeon2 pigeon = new Pigeon2(Constants.GyroId);
  
  
  public final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
/*
  public final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          RobotContainer.m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });
 */
  public final SwerveDrivePoseEstimator m_PoseEstimator = 
      new SwerveDrivePoseEstimator(
        m_kinematics, pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          }, new Pose2d(0, 0, pigeon.getRotation2d())); // right in front of speaker if auto doesn't set.


  @Override
  public void periodic() {
        updateOdometry();
    questNav.commandPeriodic();
    
  }


  public void spinFast() {
    Drivetrain.kMaxAngularSpeed = Drivetrain.kMaxAngularSpeedFastSet;
  }
  public void spinNormal() {
    Drivetrain.kMaxAngularSpeed = Drivetrain.kMaxAngularSpeedSet;
  }

  @AutoLogOutput
  public double getDegrees() {
    return pigeon.getRotation2d().getDegrees();
  }

  public void outputTelemetry() {

  }

  public Drivetrain() {

    
  }

  

  public void driveSwerve(double moveY, double moveX, double rotate, boolean fieldRelative) {

    // critical to note that the Y direction from the controller becomes the X direction on the robot and vice-versa.


    Logger.recordOutput("moveY", moveY);
    Logger.recordOutput("moveX", moveX);

    if (fieldRelative && FlipUtil.shouldFlip()) {
      moveY *= -1;
      moveX *= -1;
    }

    final var xSpeed =


            MathUtil.applyDeadband(moveY, 0.02)
            * Drivetrain.kMaxSpeed; 


    final var ySpeed =
    

            MathUtil.applyDeadband(moveX, 0.02)
            * Drivetrain.kMaxSpeed;


    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
       // m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
       //     * Drivetrain.kMaxAngularSpeed;

       MathUtil.applyDeadband(rotate, 0.02)
            * Drivetrain.kMaxAngularSpeed;

    this.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    m_PoseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  private void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    Logger.start();
    Logger.recordOutput("xspeed", xSpeed);
    Logger.recordOutput("yspeed", ySpeed);
    Logger.recordOutput("rot", rot);

    SwerveModuleState[] swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeon.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    drive(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond,
        false
    );
  }

  public void driveFieldRelative(ChassisSpeeds fieldSpeeds) {
    drive(
        fieldSpeeds.vxMetersPerSecond,
        fieldSpeeds.vyMetersPerSecond,
        fieldSpeeds.omegaRadiansPerSecond,
        true
    );
  }

  public void resetGyro() {
    pigeon.reset();
  }

 /* public void resetWheels() {
    SwerveModuleState startingState = new SwerveModuleState(0, new Rotation2d(Math.PI/2));
    
    m_frontLeft.setDesiredState(startingState);
    m_frontRight.setDesiredState(startingState);
    m_backLeft.setDesiredState(startingState);
    m_backRight.setDesiredState(startingState);
  } */

  @AutoLogOutput
  public Rotation2d getHeading() {
    return m_PoseEstimator.getEstimatedPosition().getRotation();
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    

var fl = m_frontLeft.getPosition();
var fr = m_frontRight.getPosition();
var bl = m_backLeft.getPosition();
var br = m_backRight.getPosition();

Logger.recordOutput("SwervePos/FL_m", fl.distanceMeters);
Logger.recordOutput("SwervePos/FR_m", fr.distanceMeters);
Logger.recordOutput("SwervePos/BL_m", bl.distanceMeters);
Logger.recordOutput("SwervePos/BR_m", br.distanceMeters);
Logger.recordOutput("Pose", m_PoseEstimator.getEstimatedPosition().toString());

    m_PoseEstimator.update(
        pigeon.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
         
  }
@AutoLogOutput
  public Pose2d getPose() {
    return m_PoseEstimator.getEstimatedPosition();
  }

    // Reset the pose of the swerve module.
  public void resetPose(Pose2d newFieldRelativePose) {
 
    Rotation2d gyroAngle = pigeon.getRotation2d();
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        };

    m_PoseEstimator.resetPosition(gyroAngle, modulePositions, newFieldRelativePose);
  } 

  ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_backLeft.getState(),
      m_backRight.getState()
    });
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    ChassisSpeeds fieldRelative =
    ChassisSpeeds.fromRobotRelativeSpeeds(
      getRobotRelativeSpeeds(),
      m_PoseEstimator.getEstimatedPosition().getRotation()
    );
    return fieldRelative;
  }

}