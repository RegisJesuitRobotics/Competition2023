package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.WPIUtilJNI;
import java.util.function.DoubleSupplier;

public class VectorRateLimiter {
    private final DoubleSupplier limit;
    private Translation2d lastVector;
    private double lastTime;

    public VectorRateLimiter(DoubleSupplier limit) {
        this.limit = limit;

        reset();
    }

    public VectorRateLimiter(double limit) {
        this(() -> limit);
    }

    public Translation2d calculate(Translation2d vector) {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - lastTime;
        Translation2d delta = vector.minus(lastVector);

        double currentLimit = limit.getAsDouble();
        if (delta.getNorm() > currentLimit * elapsedTime) {
            delta = new Translation2d(currentLimit * elapsedTime, delta.getAngle());
        }

        lastTime = currentTime;
        lastVector = lastVector.plus(delta);
        return lastVector;
    }

    public void reset(Translation2d translation2d) {
        lastVector = translation2d;
        lastTime = WPIUtilJNI.now() * 1e-6;
    }

    public void reset() {
        reset(new Translation2d());
    }
}
