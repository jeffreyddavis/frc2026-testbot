// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveModuleIds;
//import swervelib.parser.SwerveParser;
//import swervelib.SwerveDrive;
//import swervelib.SwerveInputStream;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.Shooter;
import gg.questnav.questnav.QuestNav;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class RobotContainer {


  
  private final Vision vision;

  private final Drivetrain m_swerve = new Drivetrain();
  private final Turret m_turret = new Turret(m_swerve.pigeon, m_swerve);
  private final Shooter m_shooter = new Shooter();
  
  public final static Joystick m_joystick = new Joystick(0);
  public final static CommandXboxController m_controller = new CommandXboxController(1);

  @AutoLogOutput
  public static double forwardAxis = 0;
  AnalogInput m_analog = new AnalogInput(0);

 // public final static SparkMax testMotor = new SparkMax(SwerveModuleIds.BackRightTurn, 
 // com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
@AutoLogOutput
  public double readAnalog() {
    return m_analog.getVoltage();
  }

  private final CommandJoystick m_driverController =
      new CommandJoystick(1);

  
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  /*
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> m_driverController.getRawAxis(1) * -1,
      () -> m_driverController.getRawAxis(0) * -1)
  .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(0.8)
  .allianceRelativeControl(true);

 */
  public RobotContainer() {
    configureBindings();
    m_swerve.resetGyro();
 vision =
    new Vision(
        m_swerve::addVisionMeasurement,
       
        new VisionIOLimelight(VisionConstants.camera0Name, m_swerve::getHeading),
        new VisionIOLimelight(VisionConstants.camera1Name, m_swerve::getHeading)
         );

        //m_driveMotor = new SparkMax(driveMotorChannel, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
//testMotor.set(-1);
    //m_joystick.button(0, new InstantCommand());
  }

  private void configureBindings() {
    //Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    //drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    m_swerve.setDefaultCommand(
      Commands.run(
          () ->
              m_swerve.driveSwerve(-m_joystick.getRawAxis(1) , -m_joystick.getRawAxis(0), -m_joystick.getRawAxis(2), true),
          m_swerve
      )
  
    );

    // Enable this to have turret lock to angle from Advantage scope.
    //m_turret.setDefaultCommand(Commands.run(() -> m_turret.updateFromDashboard(), m_turret));

    //Enable this to have the turret simply sit still.
   // m_turret.setDefaultCommand(Commands.run(() -> m_turret.stop(), m_turret));
    
    // Enable these to test the turret motor
    m_controller.a().onTrue(Commands.runOnce(() -> m_shooter.testServoForward(), m_shooter));
    m_controller.b().onTrue(Commands.runOnce(() -> m_shooter.testServoBackward(), m_shooter));

    m_controller.x().onTrue(Commands.runOnce(() -> m_shooter.testServoMiddle(), m_shooter));
 
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
