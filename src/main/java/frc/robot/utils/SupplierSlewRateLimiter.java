// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class SupplierSlewRateLimiter {
    private final DoubleSupplier rateLimit;
    private double prevVal;
    private double prevTime;

    public SupplierSlewRateLimiter(DoubleSupplier rateLimit) {
        this.rateLimit = rateLimit;
        prevVal = 0.0;
        prevTime = MathSharedStore.getTimestamp();
    }

    public double calculate(double input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - prevTime;
        double limit = rateLimit.getAsDouble();
        prevVal += MathUtil.clamp(input - prevVal, -limit * elapsedTime, limit * elapsedTime);
        prevTime = currentTime;
        return prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        prevVal = value;
        prevTime = MathSharedStore.getTimestamp();
    }
}
