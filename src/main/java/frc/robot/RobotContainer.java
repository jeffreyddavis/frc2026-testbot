// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveModuleIds;
import frc.robot.addons.FuelSim;
//import swervelib.parser.SwerveParser;
//import swervelib.SwerveDrive;
//import swervelib.SwerveInputStream;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.Hood;
import gg.questnav.questnav.QuestNav;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class RobotContainer {


  
  private final Vision vision;

  public Drivetrain m_swerve = new Drivetrain();
  private final Turret m_turret = new Turret(m_swerve.pigeon, m_swerve);
  private final Hood m_hood = new Hood();
  private final Shooter m_shooter = new Shooter(m_hood, m_turret);
  
  public final static Joystick m_joystick = new Joystick(0);
  public final static CommandXboxController m_controller = new CommandXboxController(1);
  Translation2d Hub = FlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());

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
    FuelSim.getInstance(); // gets singleton instance of FuelSim
    FuelSim.getInstance().spawnStartingFuel(); // spawns fuel in the depots and neutral zone
    
    // Register a robot for collision with fuel
      FuelSim.getInstance().registerRobot(
            100.0, // from left to right
            100.0, // from front to back
            5.0, // from floor to top of bumpers
            () -> m_swerve.m_PoseEstimator.getEstimatedPosition(), // Supplier<Pose2d> of robot pose
            () -> m_swerve.m_kinematics.toChassisSpeeds()); // Supplier<ChassisSpeeds> of field-centric chassis speeds
    
      // Register an intake to remove fuel from the field as a rectangular bounding box
      FuelSim.getInstance().registerIntake(
            0, 100, 0, 10); // (optional) Runnable called whenever a fuel is intaked
    
      FuelSim.getInstance().setSubticks( 5); // sets the number of physics iterations to perform per 20ms loop. Default = 5
    
      FuelSim.getInstance().start(); // enables the simulation to run (updateSim must still be called periodically)
      FuelSim.getInstance().stop(); // stops the simulation running (updateSim will do nothing until start is called again)


    configureBindings();

    //FuelSim.getInstance().stepSim(); // steps the simulation forward by 20ms, regardless of start/stop state
    //FuelSim.getInstance().spawnFuel(Translation3d pos, Translation3d vel); // spawns a fuel with a given position and velocity (both field centric, represented as vectors by Translation3d)
    //FuelSim.getInstance().launchFuel(LinearVelocity launchVelocity, Angle hoodAngle, Angle turretYaw, Distance launchHeight); // Spawns a fuel onto the field at the robot's position with a specified launch velocity and angles, accounting for robot movement (robot must be registered)
    //FuelSim.getInstance().clearFuel(); // clears all fuel from the field

    FuelSim.Hub.BLUE_HUB.getScore(); // get number of fuel scored in blue hub
    //m_swerve.resetGyro();
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

    m_shooter.setDefaultCommand(
      Commands.run(
    () -> {
        Translation2d pos =
            m_swerve.m_PoseEstimator.getEstimatedPosition().getTranslation();

        ChassisSpeeds speeds = m_swerve.getFieldRelativeChassisSpeeds();

        Translation2d velocity =
            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        m_shooter.target(pos, velocity, Hub, 0.15);
    },
    m_shooter
));

    // Enable this to have turret lock to angle from Advantage scope.
    //m_turret.setDefaultCommand(Commands.run(() -> m_turret.updateFromDashboard(), m_turret));

    //Enable this to have the turret simply sit still.
   // m_turret.setDefaultCommand(Commands.run(() -> m_turret.stop(), m_turret));
    
    // Enable these to test the turret motor
    m_controller.a().onTrue(Commands.runOnce(() -> m_hood.testServoForward(), m_hood));
    m_controller.b().onTrue(Commands.runOnce(() -> m_hood.testServoBackward(), m_hood));

    m_controller.x().onTrue(Commands.runOnce(() -> m_hood.testServoMiddle(), m_hood));
 
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }


}
