package frc.robot.utils.led;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.led.buffer.RaiderAddressableLEDBuffer;

public class AlternatePattern implements Pattern {
    private final double pattern1Time;
    private final double pattern2Time;
    private final Pattern pattern1;
    private final Pattern pattern2;

    public AlternatePattern(double pattern1Time, double pattern2Time, Pattern pattern1, Pattern pattern2) {
        this.pattern1Time = pattern1Time;
        this.pattern2Time = pattern2Time;
        this.pattern1 = pattern1;
        this.pattern2 = pattern2;
    }

    public AlternatePattern(double periodTime, Pattern pattern1, Pattern pattern2) {
        this(periodTime / 2, periodTime / 2, pattern1, pattern2);
    }

    public AlternatePattern(double pattern1Time, double pattern2Time, Color color1, Color color2) {
        this(pattern1Time, pattern2Time, new SolidPattern(color1), new SolidPattern(color2));
    }

    public AlternatePattern(double periodTime, Color color1, Color color2) {
        this(periodTime / 2, periodTime / 2, color1, color2);
    }

    @Override
    public void applyTo(RaiderAddressableLEDBuffer buffer, double patternTime) {
        if (patternTime % (pattern1Time + pattern2Time) < pattern1Time) {
            pattern1.applyTo(
                    buffer, patternTime - (pattern2Time * ((int) (patternTime / (pattern1Time + pattern2Time)))));
        } else {
            pattern2.applyTo(
                    buffer, patternTime - (pattern1Time * ((int) (patternTime / (pattern1Time + pattern2Time)) + 1)));
        }
    }
}
