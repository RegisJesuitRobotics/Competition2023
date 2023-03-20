package frc.robot.utils.led;

import frc.robot.utils.led.buffer.RaiderAddressableLEDBuffer;

public class SplitBufferPattern implements Pattern {
    private final int offset;
    private final Pattern pattern1;
    private final Pattern pattern2;

    public SplitBufferPattern(int offset, Pattern pattern1, Pattern pattern2) {
        this.offset = offset;
        this.pattern1 = pattern1;
        this.pattern2 = pattern2;
    }

    @Override
    public void applyTo(RaiderAddressableLEDBuffer buffer, double patternTime) {
        RaiderAddressableLEDBuffer buffer1 = buffer.split(0, offset);
        RaiderAddressableLEDBuffer buffer2 = buffer.split(offset, buffer.getLength());
        pattern1.applyTo(buffer1, patternTime);
        pattern2.applyTo(buffer2, patternTime);
    }
}
