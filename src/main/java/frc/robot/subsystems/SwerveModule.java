package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

import com.revrobotics.spark.*;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;



public class SwerveModule {

  //.0508 meters = 2 inches.
  //private static final double kWheelRadius = 0.0508;
  private static final double kWheelCircumference = 0.3191858136; // based on wheel radius above.
  private static final double kWheelInchesCircumference = 12.5663706144;
  private static final double rpmToMetersPerSecondOld = 0.005319763560078716; // based on wheel radius above AKA, circumference / 60 to go from minutes to seconds.
  private static final double rpmToMetersPerSecond = 0.00078811312; // above value / 6.75 (the gear ratio)
  //private static final int kEncoderResolution = 4096;
  private static final int kNEOEncoderResolution = 42;

  //private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  //private static final double kModuleMaxAngularAcceleration =
  //    4 * Math.PI; // radians per second squared

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  public SparkMaxConfig driveMotorConfig;
  public SparkMaxConfig turningMotorConfig;

  public final RelativeEncoder m_driveEncoder;
  public final CANcoder m_turningEncoder;
  
  private boolean isDebug = false;
  private boolean isInvertedDrive = false;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_turningPIDController = new PIDController(1, 0, 0);
 // private final PIDController m_DrivePIDController = new PIDController(1, 0, 0);
  //new PIDController(2.30769230769, 0, 0); //12volts/5.2meters per second = 2.30769230769. 1 meters per second would be this number of volts. 

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel CAN channel for the drive motor.
   * @param turningMotorChannel CAN channel for the turning motor.
   * @param turningEncoderChannel CAN channel for the turning encoder.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      boolean invertedDrive
      ) {

        if (driveMotorChannel == Constants.SwerveModuleIds.BackRightDrive) isDebug = true;
    m_driveMotor = new SparkMax(driveMotorChannel, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);


    isInvertedDrive = invertedDrive;
    driveMotorConfig = new SparkMaxConfig();
    driveMotorConfig.inverted(false);
    driveMotorConfig.idleMode(IdleMode.kBrake);
    driveMotorConfig.openLoopRampRate(0);
    
    m_driveMotor.configure(driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_turningMotor = new SparkMax(turningMotorChannel, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPosition(0);
    m_turningEncoder = new CANcoder(turningEncoderChannel);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(0, Math.PI*2);
  }


  /**
   * Calculates the distance turned in radians.
   *
   * @return The distance turned in radians.
   */


  public double TurningDistance() {
    double angle = m_turningEncoder.getAbsolutePosition().getValueAsDouble()  * (2 * Math.PI );

    if (isDebug) {
      Logger.recordOutput("Turning encoder", m_turningEncoder.getAbsolutePosition().getValueAsDouble());
      
    }

    // we offset each encoder so that 0 is forward.

    // this is offset code for when the encoders are 90 degress off.
    //angle += (Math.PI / 2); // offset by 90 degrees
    //if (angle >= (2*Math.PI)) angle -= (2*Math.PI); // wrap anything over 360 degrees. 
    return angle;
  }

  public double TurningDegrees() {
    double angle =  m_turningEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    if (isDebug) Logger.recordOutput("turning degrees", angle);
    return angle;
  }

  /**
   * Calculates the distance driven in meters.
   *
   * @return The distance driven in meters.
   */
  @AutoLogOutput
  public double DrivenDistance() {

if (isDebug) Logger.recordOutput("drive encoder", m_driveEncoder.getPosition());

    return -((m_driveEncoder.getPosition() * kWheelCircumference) / (6.75 / kNEOEncoderResolution));
    

    //
  }

  public double InchesDistance() {
    return -((m_driveEncoder.getPosition() * kWheelInchesCircumference) * (6.75 / kNEOEncoderResolution));
  }

 /**
   * Calculates the current speed in meters per second.
   *
   * @return The speed in meters per second.
   */
  private double speedInMetersPerSecond() {
    return m_driveEncoder.getVelocity() * rpmToMetersPerSecond;
    
  } 

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {    
    return new SwerveModuleState(
      speedInMetersPerSecond(), new Rotation2d(TurningDistance()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      DrivenDistance(), new Rotation2d(TurningDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */

  @AutoLogOutput
  public void setDesiredState(SwerveModuleState desiredState) {

  
    var encoderRotation = new Rotation2d(TurningDistance());


    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState,encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the turning motor output from the turning PID controller.
    // modules are clockwise, so invert the value. 
    final double turnOutput =
        -m_turningPIDController.calculate(TurningDistance(), state.angle.getRadians());

//    final double turnFeedforward =
//        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    //m_driveMotor.set(state.speedMetersPerSecond/5.2); // Max speed is apparently 5.2 meters per second (per testing). This seems fast ¯\_(ツ)_/¯
    // almost exactly 17 feet per second
    // This number should be adjusted with additional testing. We may need to scale it.
    // I think a feedforward could make this more accurate. HMMMM.

    //final double driveOutput = m_DrivePIDController.calculate(speedInMetersPerSecond(), state.speedMetersPerSecond);
    //final double driveOutput = 12 * state.speedMetersPerSecond;

    // 12volts/5.2meters per second = 2.30769230769. 1 meters per second would be this number of volts. (Set in the PID Controller)
    //m_driveMotor.set(-state.speedMetersPerSecond/5.2);

if (isDebug) {
    Logger.recordOutput("turning radians", TurningDistance());
  Logger.recordOutput("speed output", state.speedMetersPerSecond);
  Logger.recordOutput("turn output", turnOutput);
  }

    double outputPower = -(state.speedMetersPerSecond*12)/5.2;
    if (isInvertedDrive) outputPower *= -1;

    m_driveMotor.setVoltage(outputPower);
    // Note that max voltage of 12 means that a turn rate of 1 * P of 2 means we max turn at 1/6th speed. This seems to be fast enough though so... great! :D
    m_turningMotor.setVoltage(turnOutput/*  + turnFeedforward*/);  

    //SmartDashboard.putNumber("turn output", turnOutput);
  }
}
