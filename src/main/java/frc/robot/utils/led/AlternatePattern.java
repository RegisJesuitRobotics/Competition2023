package frc.robot.utils.led;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.led.buffer.RaiderAddressableLEDBuffer;

public class AlternatePattern implements Pattern {
    private final double period;
    private final Pattern pattern1;
    private final Pattern pattern2;

    public AlternatePattern(double period, Pattern pattern1, Pattern pattern2) {
        this.period = period;
        this.pattern1 = pattern1;
        this.pattern2 = pattern2;
    }

    public AlternatePattern(double period, Color color1, Color color2) {
        this(period, new SolidPattern(color1), new SolidPattern(color2));
    }

    @Override
    public void applyTo(RaiderAddressableLEDBuffer buffer, double patternTime) {
        if (patternTime % period < period / 2.0) {
            pattern1.applyTo(buffer, patternTime);
        } else {
            pattern2.applyTo(buffer, patternTime - (period / 2.0));
        }
    }
}
