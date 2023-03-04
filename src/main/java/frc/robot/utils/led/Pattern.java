package frc.robot.utils.led;

import frc.robot.utils.led.buffer.RaiderAddressableLEDBuffer;

public interface Pattern {
    void applyTo(RaiderAddressableLEDBuffer buffer, double patternTime);
}
