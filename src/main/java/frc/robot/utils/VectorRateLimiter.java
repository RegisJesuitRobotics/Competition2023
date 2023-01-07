package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.WPIUtilJNI;

public class VectorRateLimiter {
    private final double limit;
    private Translation2d lastVector;
    private double lastTime;

    public VectorRateLimiter(double limit) {
        this.limit = limit;

        reset();
    }

    public Translation2d calculate(Translation2d vector) {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - lastTime;
        Translation2d delta = vector.minus(lastVector);

        if (delta.getNorm() > limit * elapsedTime) {
            delta = new Translation2d(limit * elapsedTime, delta.getAngle());
        }

        lastTime = currentTime;
        lastVector = lastVector.plus(delta);
        return lastVector;
    }

    public void reset() {
        lastVector = new Translation2d();
        lastTime = WPIUtilJNI.now() * 1e-6;
    }
}
