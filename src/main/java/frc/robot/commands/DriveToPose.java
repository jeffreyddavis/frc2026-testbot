package frc.robot.commands;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.Constants.Driving;
import frc.robot.addons.PIDMint;
import frc.robot.subsystems.Drivetrain;

public class DriveToPose extends Command {

  private final PIDMint xController;
  private final PIDMint yController;
  private final PIDMint rotController;

  private final Timer stopTimer = new Timer();
  private final Timer dontSeeTagTimer = new Timer();

  private final Drivetrain drivetrain;
  private final CommandJoystick driverController;
  private final double toleranceLevel;

  @AutoLogOutput
  private final Pose2d targetPose;

  public DriveToPose(
      Pose2d targetPose,
      Drivetrain drivetrain,
      CommandJoystick driverController,
      double toleranceLevel
  ) {
    this.targetPose = targetPose;
    this.drivetrain = drivetrain;
    this.driverController = driverController;
    this.toleranceLevel = toleranceLevel;

    xController = new PIDMint(
        Driving.drive_P,
        0.1, 0,
        Driving.X_TOLERANCE * toleranceLevel,
        0
    );

    yController = new PIDMint(
        Driving.drive_P,
        0.1, 0,
        Driving.Y_TOLERANCE * toleranceLevel,
        0
    );

    rotController = new PIDMint(
        Driving.turn_P,
        .03, 0,
        Driving.ROT_TOLERANCE,
        0
    );

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    stopTimer.reset();
    stopTimer.start();

    dontSeeTagTimer.reset();
    dontSeeTagTimer.start();

    rotController.enableContinuousInput(-180, 180);
    rotController.setSetpoint(targetPose.getRotation().getDegrees());

    if (toleranceLevel >= 2) {
      rotController.setTolerance(
          Driving.ROT_TOLERANCE * (toleranceLevel / 2)
      );
    } else {
      rotController.setTolerance(Driving.ROT_TOLERANCE);
    }

    xController.setSetpoint(targetPose.getX());
    xController.setTolerance(
        Driving.X_TOLERANCE * toleranceLevel
    );

    yController.setSetpoint(targetPose.getY());
    yController.setTolerance(
        Driving.Y_TOLERANCE * toleranceLevel
    );
  }

  @Override
  public void execute() {
    dontSeeTagTimer.reset();

    Pose2d current = drivetrain.getPose();

    double xSpeed = xController.calculate(current.getX());
    double ySpeed = -yController.calculate(current.getY());
    double rotSpeed = rotController.calculate(current.getRotation().getDegrees());

    // Manual override blending
    xSpeed -= driverController.getRawAxis(1);
    ySpeed -= driverController.getRawAxis(0);
    rotSpeed -= driverController.getRawAxis(2);

    drivetrain.driveFieldRelative(
        new ChassisSpeeds(
            xSpeed,
            -ySpeed,
            rotSpeed
        )
    );

    if (!xController.atSetpoint()
        || !yController.atSetpoint()
        || !rotController.atSetpoint()) {
      stopTimer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.driveRobotRelative(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return
        stopTimer.hasElapsed(Driving.poseLockTime)
        || Math.abs(driverController.getRawAxis(0)) > 0.3
        || Math.abs(driverController.getRawAxis(1)) > 0.3
        || Math.abs(driverController.getRawAxis(2)) > 0.3
        || driverController.button(1).getAsBoolean();
  }
}
