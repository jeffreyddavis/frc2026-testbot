// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;

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

import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.QuestNaviman;
import frc.robot.subsystems.RobotHealth;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class RobotContainer {


  
  private final Vision m_vision;

  public Drivetrain m_swerve = new Drivetrain();
  private final Turret m_turret = new Turret(m_swerve.pigeon, m_swerve);
  private final Hood m_hood = new Hood();
  // private final Shooter m_shooter = new Shooter(m_hood, m_turret);
  public final QuestNaviman m_QuestNaviman = new QuestNaviman(m_swerve);
  private final RobotHealth m_RobotHealth;;
  
  public final static Joystick m_joystick = new Joystick(0);
  public final static CommandXboxController m_controller = new CommandXboxController(1);
  Translation2d Hub = FlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());

  @AutoLogOutput
  public static double forwardAxis = 0;
  AnalogInput m_analog = new AnalogInput(0);


@AutoLogOutput
  public double readAnalog() {
    return m_analog.getVoltage();
  }

  private final CommandJoystick m_driverController =
      new CommandJoystick(1);

  public FuelSim fuelSim; // creates a new fuelSim of FuelSim


  public void configureFuelSim() {
    fuelSim = new FuelSim();
    fuelSim.spawnStartingFuel();

    fuelSim.start();
    SmartDashboard.putData(Commands.runOnce(() -> {
                fuelSim.clearFuel();
                fuelSim.spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));
}

private void configureFuelSimRobot(BooleanSupplier ableToIntake, Runnable intakeCallback) {
  /*
    fuelSim.registerRobot(
            Dimensions.FULL_WIDTH.in(Meters),
            Dimensions.FULL_LENGTH.in(Meters),
            Dimensions.BUMPER_HEIGHT.in(Meters),
            drive::getPose,
            drive::getFieldSpeeds);
    fuelSim.registerIntake(
            -Dimensions.FULL_LENGTH.div(2).in(Meters),
            Dimensions.FULL_LENGTH.div(2).in(Meters),
            -Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
            -Dimensions.FULL_WIDTH.div(2).in(Meters),
            () -> intake.isRightDeployed() && ableToIntake.getAsBoolean(),
            intakeCallback);
    fuelSim.registerIntake(
            -Dimensions.FULL_LENGTH.div(2).in(Meters),
            Dimensions.FULL_LENGTH.div(2).in(Meters),
            Dimensions.FULL_WIDTH.div(2).in(Meters),
            Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
            () -> intake.isLeftDeployed() && ableToIntake.getAsBoolean(),
            intakeCallback);
             */
}

  public RobotContainer() {
configureBindings();

    m_vision =
    new Vision(
        m_swerve::addVisionMeasurement,
       
        new VisionIOLimelight(VisionConstants.camera0Name, m_swerve::getHeading),
        new VisionIOLimelight(VisionConstants.camera1Name, m_swerve::getHeading)
         );

         m_RobotHealth = new RobotHealth(m_swerve, m_QuestNaviman, m_vision);

  }



  private void configureBindings() {

    m_swerve.setDefaultCommand(
      Commands.run(
          () ->
              m_swerve.driveSwerve(-m_joystick.getRawAxis(1) , -m_joystick.getRawAxis(0), -m_joystick.getRawAxis(2), true),
          m_swerve
      )
  
    );

    // continiously target the hub for testing. 

    /*
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
 */

 // m_shooter.setDefaultCommand(Commands.run(() -> m_shooter.TestShot(), m_shooter));

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
