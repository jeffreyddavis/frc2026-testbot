// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants
{
  public static final boolean disableHAL = false;
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  public static final double YawWarningTolerance = 2;
  public static final double TrenchDangerDistance = .65;
  public static final double ZONE_HYSTERESIS = 0.15;  // meters (~6 in)

  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  
  public static class SwerveModuleIds {
    public static final int FrontLeftDrive = 2;
    public static final int FrontLeftTurn = 3;
    public static final int FrontLeftEncoder = 4;

    public static final int FrontRightDrive = 5;
    public static final int FrontRightTurn = 6;
    public static final int FrontRightEncoder = 7;

    public static final int BackLeftDrive = 11;
    public static final int BackLeftTurn = 12;
    public static final int BackLeftEncoder = 10;

    public static final int BackRightDrive = 8;
   
    public static final int BackRightTurn = 9;
    public static final int BackRightEncoder = 13;
  
    
  }

  public static final class Turret {
    public static final int Motor = 20;
    public static final int Encoder = 21;
    public static final double TestSpeed = .1;
    public static final double EncoderOffset = 0;
    public static final double FORBIDDEN_LIMIT_DEG = 180.0;
    public static final double FORBIDDEN_BUFFER_DEG = 5.0; // safety margin    
    public static final double GEAR_RATIO = 45.0; // example

  }

  public static final class Shooter {
    public static final int ShooterRight = 35; // negative is shoot
    public static final int ShooterLeft = 36; // positive is shoot
  }

  public static final int GyroId = 14;

  public static final class Driving {
    public static final double drive_P = 1;
    public static final double turn_P = 1;

    public static final double X_TOLERANCE = 1;
    public static final double Y_TOLERANCE = 1;
    public static final double ROT_TOLERANCE = 1;

    public static final double poseLockTime = .1;
  }

  public static final class Hood {
    public static final double MIN_HOOD_DEG = 0;
    public static final double MAX_HOOD_DEG = 90;
    public static final double MIN_MEANINGFUL_DISTANCE = 0;
    public static final double MAX_MEANINGFUL_DISTANCE = 50;
  }
}

