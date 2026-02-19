package frc.robot.subsystems;

import javax.xml.crypto.dsig.TransformService;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Hood;
import frc.robot.Constants;

import frc.robot.addons.HoodParams;

public class Shooter extends SubsystemBase {

    private Hood m_Hood;
    private Turret m_Turret;

    @AutoLogOutput
    private double hoodTarget;
    @AutoLogOutput
    private double turretTarget;
    @AutoLogOutput
    private double targetDistance;
    @AutoLogOutput
    private double requiredVelocity;
    @AutoLogOutput
    private double rawHoodTOFFromDistance;

    public Shooter(Hood hood, Turret turret) {
        m_Hood = hood;
        m_Turret = turret;
    }
        private static final InterpolatingTreeMap<Double, HoodParams> HOOD_MAP =
            new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(),
                (a, b, t) -> a.interpolate(b, t)
            );
        static {
            
          // degrees, seconds
          HOOD_MAP.put(0.5, new HoodParams(69, 0.2));
        HOOD_MAP.put(1.0, new HoodParams(52.4, 0.28));
        HOOD_MAP.put(1.5, new HoodParams(40.8, 0.38));  // degrees, seconds
        HOOD_MAP.put(2.0, new HoodParams(32.9, 0.45));
        HOOD_MAP.put(2.5, new HoodParams(27.8, 0.52));
        HOOD_MAP.put(3.0, new HoodParams(23.4, 0.60));
        HOOD_MAP.put(3.5, new HoodParams(30.0, 0.68));
        HOOD_MAP.put(4.0, new HoodParams(58.0, 0.76));
        HOOD_MAP.put(4.5, new HoodParams(61.0, 0.85));
        HOOD_MAP.put(5.0, new HoodParams(64.0, 0.94));
    }

    public double getTotalVelocity(double distance) {
        HoodParams params = getParams(distance);
        double vHoriz = distance / params.timeOfFlightSeconds();
        return vHoriz / Math.cos(Math.toRadians(params.degrees()));
    }

    public double effectiveHood (double distance, double requiredVelocity) {
        HoodParams params = getParams(distance);

        double vHoriz = distance / params.timeOfFlightSeconds();
        // Solve for new hood angle
        double ratio = MathUtil.clamp(
            vHoriz / requiredVelocity,
            -1.0,
            1.0
        );
        double adjustedHood = Math.toDegrees(Math.acos(ratio));
        adjustedHood = MathUtil.clamp(adjustedHood, Constants.Hood.MIN_HOOD_DEG, Constants.Hood.MAX_HOOD_DEG);

        return adjustedHood;
        
        
    }

    public HoodParams getParams(double distance) {
        double clampedDistance = MathUtil.clamp(
            distance,
            Constants.Hood.MIN_MEANINGFUL_DISTANCE,
            Constants.Hood.MAX_MEANINGFUL_DISTANCE
        );

        
        return HOOD_MAP.get(clampedDistance);
    }
    

    @AutoLogOutput
    Translation2d m_robotPosition;
    @AutoLogOutput
    Translation2d m_robotVelocity;
    @AutoLogOutput
    Translation2d m_goalPosition;

    public void target(
            Translation2d robotPosition,
            Translation2d robotVelocity,
            Translation2d goalPosition,
            double latencyCompensation
        ) {

            m_robotPosition = robotPosition;
            m_robotVelocity = robotVelocity;
            m_goalPosition = goalPosition;
            // 1. Project future position
            Translation2d futurePos = robotPosition.plus(
                robotVelocity.times(latencyCompensation) 
            );

            // 2. Get target vector
            Translation2d toGoal = goalPosition.minus(futurePos);
            targetDistance = toGoal.getNorm();
            if (targetDistance < 1e-3) {
                return;
            }
            Translation2d targetDirection = toGoal.div(targetDistance);

            // 3. Look up baseline velocity from table

            HoodParams baseline = getParams(targetDistance);
            rawHoodTOFFromDistance = baseline.timeOfFlightSeconds();
            double baselineVelocity = targetDistance / rawHoodTOFFromDistance;

            // 4. Build target velocity vector
            Translation2d targetVelocity = targetDirection.times(baselineVelocity);

            // 5. THE MAGIC: subtract robot velocity
            Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

            // 6. Extract results
            Rotation2d turretAngle = shotVelocity.getAngle();
            requiredVelocity = shotVelocity.getNorm();



            // 7. Adjust hood angle to compensate for required shot velocity
            //hoodTarget = effectiveHood(targetDistance, requiredVelocity);
            hoodTarget = baseline.degrees();
            turretTarget = turretAngle.getDegrees();



            m_Hood.setPositionAngle(hoodTarget);
            m_Turret.setFieldTargetAngle(turretAngle.getDegrees());

        }
    }
