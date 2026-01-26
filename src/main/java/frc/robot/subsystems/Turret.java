package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Turret extends SubsystemBase {

    private final TalonFX m_motor = new TalonFX(Constants.Turret.Motor);
    private final CANcoder m_encoder = new CANcoder(Constants.Turret.Encoder);

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

    public Turret() {
        configureMotor();
        configureEncoder();
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

    public void updateFromDashboard() {
        double target = targetAngleDeg.get();
        goToAngleDegrees(target);
    }

    private void setPercentOutput(double percent) {
        voltageOut.Output = 12 * percent;
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

        setPercentOutput(-delta/360);
        // Direction only (open-loop test)

    }

}
