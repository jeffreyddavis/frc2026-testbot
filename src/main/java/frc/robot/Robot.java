// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.VisionIOLimelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {

    Logger.recordMetadata("ProjectName", "testbot"); // Set a metadata value

    SmartDashboard.putBoolean("Calibrate Limelights", PreLimeLimelights);

    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.


    m_robotContainer = new RobotContainer();

  }

  @AutoLogOutput
  private boolean PreLimeLimelights;

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    LimelightHelpers.SetRobotOrientation("limelight-right", 0, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-left", 0, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-turret", 0, 0, 0, 0, 0, 0);

  }

  @Override
  public void disabledInit() {
    LimelightHelpers.setPipelineIndex("limelight-right", 1);
    LimelightHelpers.setPipelineIndex("limelight-left", 1);
    LimelightHelpers.setPipelineIndex("limelight-turret", 1);
  }

  @Override
  public void disabledPeriodic() {
    PreLimeLimelights = SmartDashboard.getBoolean("Calibrate Limelights", PreLimeLimelights);
    
    if (PreLimeLimelights) {


      LimelightHelpers.setPipelineIndex("limelight-right", 0);
      LimelightHelpers.setPipelineIndex("limelight-left", 0);
      LimelightHelpers.setPipelineIndex("limelight-turret", 0);

      LimelightHelpers.SetIMUMode("limelight-right", 1); // Seed internal IMU
      LimelightHelpers.SetIMUMode("limelight-left", 1); // Seed internal IMU
      LimelightHelpers.SetIMUMode("limelight-turret", 1); // Seed internal IMU


    }

    else {
      LimelightHelpers.setPipelineIndex("limelight-right", 1);
      LimelightHelpers.setPipelineIndex("limelight-left", 1);
      LimelightHelpers.setPipelineIndex("limelight-turret", 1);
    }


  }

  @Override
  public void disabledExit() {
    LimelightHelpers.setPipelineIndex("limelight-right", 0);
    LimelightHelpers.setPipelineIndex("limelight-left", 0);
    LimelightHelpers.setPipelineIndex("limelight-turret", 0);

    //m_robotContainer.m_swerve.pigeon.setYaw(m_robotContainer.m_swerve.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees());


    LimelightHelpers.SetIMUMode("limelight-right", 0); // Use internal IMU + external IMU
    LimelightHelpers.SetIMUMode("limelight-left", 0); // Use internal IMU + external IMU
    LimelightHelpers.SetIMUMode("limelight-turret", 0); // Use internal IMU + external IMU
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }



  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.readAnalog();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {

  }

  @Override
  public void simulationPeriodic() {
    m_robotContainer.fuelSim.updateSim();
}
}
