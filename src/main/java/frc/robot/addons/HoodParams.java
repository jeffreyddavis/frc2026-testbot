package frc.robot.addons;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.MathUtil;


public record HoodParams(double degrees, double timeOfFlightSeconds)
        implements Interpolatable<HoodParams> {

    @Override
    public HoodParams interpolate(HoodParams endValue, double t) {
        return new HoodParams(
            MathUtil.interpolate(this.degrees, endValue.degrees, t),
            MathUtil.interpolate(this.timeOfFlightSeconds, endValue.timeOfFlightSeconds, t)
        );
    }
}
