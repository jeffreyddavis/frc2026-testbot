package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import frc.robot.FieldConstants;
import frc.robot.FlipUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Turret extends SubsystemBase {

    private final TalonFX m_motor = new TalonFX(Constants.Turret.Motor);
    private final CANcoder m_encoder = new CANcoder(Constants.Turret.Encoder);
    private Pigeon2 m_gyro;
    private Drivetrain m_drivetrain;
    private final LoggedNetworkNumber targetAngleDeg =
        new LoggedNetworkNumber("Turret/TargetAngleDeg", 0.0);

    private final LoggedNetworkNumber currentAngleDeg =
        new LoggedNetworkNumber("Turret/CurrentAngleDeg", 0.0);

    //private final LoggedNetworkNumber currentAngleSignedDeg =
    //    new LoggedNetworkNumber("Turret/CurrentAngleSignedDeg", 0.0);

    private final LoggedNetworkNumber deltaAngleDeg =
        new LoggedNetworkNumber("Turret/DeltaAngleDeg", 0.0);


    private final LoggedNetworkNumber forbiddenStop =
        new LoggedNetworkNumber("Turret/ForbiddenStop", 0.0);


    // Cached control object (avoids garbage)
    private final VoltageOut voltageOut = new VoltageOut(0);

    public Turret(Pigeon2 gyro, Drivetrain drivetrain) {

        configureMotor();
        configureEncoder();
        m_gyro = gyro;
        m_drivetrain = drivetrain;
    }

    /* ===================== CONFIG ===================== */

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // For testing: coast is usually nicer
        config.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast;

        // Optional: current limit to protect things during testing
        config.CurrentLimits.SupplyCurrentLimit = 25;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_motor.getConfigurator().apply(config);
    }

    private void configureEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        // Absolute encoder setup
        config.MagnetSensor.SensorDirection =
            com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive;

        // Adjust this once you know turret "forward"
        config.MagnetSensor.MagnetOffset = Constants.Turret.EncoderOffset;

        m_encoder.getConfigurator().apply(config);
    }

    /* ===================== TEST METHODS ===================== */

    /** Rotate turret clockwise at a fixed speed */
    public void testClockwise() {
        setPercentOutput(Constants.Turret.TestSpeed);
    }

    /** Rotate turret counter-clockwise at a fixed speed */
    public void testCounterClockwise() {
        setPercentOutput(-Constants.Turret.TestSpeed);
    }

    /** Stop turret motion */
    public void stop() {
        setPercentOutput(0);
    }
    
    
    public double translateDegreesfromRobot(double targetDegrees){
        Logger.recordOutput("translate/targetDegrees", targetDegrees);
        double drivetrainDegrees = m_drivetrain.getPose().getRotation().getDegrees();
        Logger.recordOutput("translate/driveTrain Degrees", drivetrainDegrees);
        double translated = targetDegrees - drivetrainDegrees;
        Logger.recordOutput("translate/translated", translated);
        return translated;
    }

    public static double getAngleBetween(Translation2d v1, Translation2d v2) {
        Translation2d difference = v2.minus(v1);

        // Get the angle of the difference vector using its components
        // atan2 in Java takes y first, then x
        double angleRadians = Math.atan2(difference.getY(), difference.getX());

        // You can also use the getAngle() method provided by Translation2d
        Rotation2d angleRotation2d = difference.getAngle();
        angleRadians = angleRotation2d.getRadians();

        // To get degrees:
        double angleDegrees = Math.toDegrees(angleRadians);
        return angleDegrees;

        /* 
        // Calculate the dot product manually: A.x * B.x + A.y * B.y
        double dotProduct = v1.getX() * v2.getX() + v1.getY() * v2.getY();

        // Calculate the magnitudes (norms)
        double magnitude1 = v1.getNorm();
        double magnitude2 = v2.getNorm();

        // Use Math.acos to find the angle in radians
        // Ensure the denominator is not zero to avoid errors
        if (magnitude1 == 0 || magnitude2 == 0) {
            return 0.0; // Or throw an exception, depending on desired behavior
        }
        
        double cosAngle = dotProduct / (magnitude1 * magnitude2);
        // Clamp the value to the range [-1, 1] to avoid issues with floating point inaccuracies
        cosAngle = Math.max(-1.0, Math.min(1.0, cosAngle));
        
        double radians = Math.acos(cosAngle); // Returns angle in radians
        radians += Math.PI; // convert conventions 

        return - ( radians * (180/ Math.PI));
        */
    }

    public void pointatHub() {
        Translation2d Hub = FlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());

        Logger.recordOutput("pointatHub/Hub Location", Hub);
        Translation2d Robot = m_drivetrain.getPose().getTranslation();

        Logger.recordOutput("pointatHub/Robot Location", Robot);

        double angleBeteen = getAngleBetween(Robot,Hub );
        Logger.recordOutput("pointatHub/Angle between", angleBeteen);

        double targetDegrees = translateDegreesfromRobot(angleBeteen);
                Logger.recordOutput("pointatHub/Hub Degrees", targetDegrees);
                
        goToAngleDegrees(targetDegrees); // enable this to test going to target
    }
    public void updateFromDashboard() {
        double target = targetAngleDeg.get();
        target = translateDegreesfromRobot(target);
        goToAngleDegrees(target);
    }

    private void setPercentOutput(double percent) {
        double IntendedOutput = percent * 3;
        if (Math.abs(IntendedOutput) > .3) IntendedOutput = .3 * Math.signum(IntendedOutput);
        voltageOut.Output = (12 * percent);
        m_motor.setControl(voltageOut);
    }
    
    
    /* ===================== ENCODER ===================== */

    /** @return turret angle in degrees -180 to 180) */
    public double getTurretAngleDegrees() {
        return m_encoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    /** @return turret angle in radians */
    public double getTurretAngleRadians() {
        return Units.degreesToRadians(getTurretAngleDegrees());
    }

    private double normalizeToSignedDegrees(double degrees) {
        double angle = degrees % 360.0;
        while (angle >= 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    private boolean wouldCrossForbidden(double currentSignedDeg, double commandedDelta) {
        // Near +180 and trying to increase angle
        if (currentSignedDeg > (Constants.Turret.FORBIDDEN_LIMIT_DEG - Constants.Turret.FORBIDDEN_BUFFER_DEG)
            && commandedDelta > 0) {
            return true;
        }

        // Near -180 and trying to decrease angle
        if (currentSignedDeg < (-Constants.Turret.FORBIDDEN_LIMIT_DEG + Constants.Turret.FORBIDDEN_BUFFER_DEG)
            && commandedDelta < 0) {
            return true;
        }

        return false;
    }


    private double computeSafeDelta(double currentDeg, double targetDeg) {
        double target  = normalizeToSignedDegrees(targetDeg);
        
        double maxAllowed = 180 - Constants.Turret.FORBIDDEN_BUFFER_DEG;
        if (Math.abs(target) > maxAllowed) target = Math.signum(target) * maxAllowed;
        double delta = target - currentDeg;



        // If the shortest path crosses the forbidden boundary, take the long way
        boolean crossesForbidden = wouldCrossForbidden(currentDeg, delta);

        if (crossesForbidden) {
            // Take long way around
            if (delta > 0) {
                delta -= 360;
            } else {
                delta += 360;
            }
        }

        return delta;
    }

    public void periodic() {
        double current = getTurretAngleDegrees();
        currentAngleDeg.set(current);
        pointatHub();
    }


    /* ===================== FUTURE PID HOOK ===================== */

    public void goToAngleDegrees(double targetDegrees) {
        double current = getTurretAngleDegrees();
        double delta = computeSafeDelta(current, targetDegrees);

       // currentAngleSignedDeg.set(currentSigned);
        deltaAngleDeg.set(delta);

        if (Math.abs(delta) < 5.0) {
            stop();
            return;
        }


        

        boolean forbidden = wouldCrossForbidden(current, delta);
        forbiddenStop.set(forbidden ? 1.0 : 0.0);
        // HARD SAFETY CHECK: never cross ±180
        if (forbidden) {
            stop();
            return;
        }

        setPercentOutput((-delta*2)/360);
        // Direction only (open-loop test)

    }

}
