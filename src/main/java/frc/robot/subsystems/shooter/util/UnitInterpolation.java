package frc.robot.subsystems.shooter.util;

import java.util.Comparator;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

// https://github.com/FRC1257/2026-Robot/blob/Sam's-Branch/src/main/java/frc/robot/subsystems/Shooter/ShooterTrajectoryCalculator.java
public final class UnitInterpolation {
    public static <M extends Measure<?>> InverseInterpolator<M> inverseInterpolate() {
			return (start, end, query) -> {
				double startBase = start.baseUnitMagnitude();
				double endBase = end.baseUnitMagnitude();
				double queryBase = query.baseUnitMagnitude();

				if(endBase - startBase <= 0) return 0;

				return Math.max(0.0, (queryBase - startBase) / (endBase - startBase));
			};
    }

    @SuppressWarnings("unchecked")
    public static <M extends Measure<?>> Interpolator<M> Interpolator(Unit unit) {
			Interpolator<M> interpolator = (start, end, t) -> (M) unit.ofBaseUnits(
				start.baseUnitMagnitude() + t*(end.baseUnitMagnitude()-start.baseUnitMagnitude())
		);
        
		return interpolator;
	}

	public static <M extends Measure<?>> Comparator<M> comparator(){
		return (a, b) -> Double.compare(a.baseUnitMagnitude(), b.baseUnitMagnitude());
	}
}
