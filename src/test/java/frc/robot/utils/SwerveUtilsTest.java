package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class SwerveUtilsTest {
    @Test
    void calculateContinuousInputSetpoint_0_0_0() {
        assertEquals(0, SwerveUtils.calculateContinuousInputSetpoint(0, 0));
    }

    @Test
    void calculateContinuousInputSetpoint_2PI_0_2PI() {
        assertEquals(Math.PI * 2, SwerveUtils.calculateContinuousInputSetpoint(Math.PI * 2, 0));
    }

    @Test
    void calculateContinuousInputSetpoint_0_2PI_0() {
        assertEquals(0, SwerveUtils.calculateContinuousInputSetpoint(0, Math.PI * 2));
    }

    @Test
    void calculateContinuousInputSetpoint_3PI_PI_3PI() {
        assertEquals(Math.PI * 3, SwerveUtils.calculateContinuousInputSetpoint(Math.PI * 3, Math.PI));
    }

    @Test
    void applyCircleDeadZone_InsideCircle_NoChange() {
        assertEquals(new Translation2d(0.5, 0.5), SwerveUtils.applyCircleDeadZone(new Translation2d(0.5, 0.5), 1.0));
    }

    @Test
    void applyCircleDeadZone_InsideCircle_NonOneMax_NoChange() {
        assertEquals(new Translation2d(1.0, 1.5), SwerveUtils.applyCircleDeadZone(new Translation2d(1.0, 1.5), 10.0));
    }

    private static final double rootTwoOverTwo = Math.sqrt(2.0) / 2;

    @Test
    void applyCircleDeadZone_OutsideCircle_Normalized() {
        assertEquals(
                new Translation2d(rootTwoOverTwo, rootTwoOverTwo),
                SwerveUtils.applyCircleDeadZone(new Translation2d(1.0, 1.0), 1.0));
    }

    @Test
    void applyCircleDeadZone_OutsideCircle_Negative_Normalized() {
        assertEquals(
                new Translation2d(-rootTwoOverTwo, rootTwoOverTwo),
                SwerveUtils.applyCircleDeadZone(new Translation2d(-1.0, 1.0), 1.0));
    }
}
