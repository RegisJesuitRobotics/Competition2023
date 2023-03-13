package frc.robot.utils.led;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.led.buffer.RaiderAddressableLEDBuffer;

public class SlidePattern implements Pattern {
    private final double period;
    private final boolean reverse;
    private final Pattern toSlidePattern;
    private final Pattern startingPattern;

    public SlidePattern(double period, boolean reverse, Pattern toSlidePattern, Pattern startingPattern) {
        this.period = period;
        this.toSlidePattern = toSlidePattern;
        this.startingPattern = startingPattern;
        this.reverse = reverse;
    }

    public SlidePattern(double period, Pattern toSlidePattern, Pattern startingPattern) {
        this(period, false, toSlidePattern, startingPattern);
    }

    public SlidePattern(double period, boolean reverse, Color startColor, Color endColor) {
        this(period, reverse, new SolidPattern(startColor), new SolidPattern(endColor));
    }

    public SlidePattern(double period, Color startColor, Color endColor) {
        this(period, false, startColor, endColor);
    }

    @Override
    public void applyTo(RaiderAddressableLEDBuffer buffer, double patternTime) {
        int length = buffer.getLength();
        int offset = (int) (patternTime % period / period * length);
        RaiderAddressableLEDBuffer buffer1, buffer2;
        if (reverse) {
            buffer1 = buffer.split(offset, length);
            buffer2 = buffer.split(0, offset);
        } else {
            buffer1 = buffer.split(0, offset);
            buffer2 = buffer.split(offset, length);
        }
        toSlidePattern.applyTo(buffer1, patternTime);
        startingPattern.applyTo(buffer2, patternTime);
    }
}
