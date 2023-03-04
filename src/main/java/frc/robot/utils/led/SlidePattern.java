package frc.robot.utils.led;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.led.buffer.RaiderAddressableLEDBuffer;

public class SlidePattern implements Pattern {

    private final double period;
    private final Pattern pattern1;
    private final Pattern pattern2;

    public SlidePattern(double period, Pattern pattern1, Pattern pattern2) {
        this.period = period;
        this.pattern1 = pattern1;
        this.pattern2 = pattern2;
    }

    public SlidePattern(double period, Color color1, Color color2) {
        this(period, new SolidPattern(color1), new SolidPattern(color2));
    }

    @Override
    public void applyTo(RaiderAddressableLEDBuffer buffer, double patternTime) {
        int length = buffer.getLength();
        int offset = (int) (patternTime % period / period * length);
        RaiderAddressableLEDBuffer buffer1 = buffer.split(0, offset);
        RaiderAddressableLEDBuffer buffer2 = buffer.split(offset, length);
        pattern1.applyTo(buffer1, patternTime);
        pattern2.applyTo(buffer2, patternTime);
    }
}
