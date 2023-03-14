package frc.robot.utils.led;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.led.buffer.RaiderAddressableLEDBuffer;

public class SolidPattern implements Pattern {
    private final Color color;

    public SolidPattern(Color color) {
        this.color = color;
    }

    @Override
    public void applyTo(RaiderAddressableLEDBuffer buffer, double patternTime) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }

    @Override
    public String toString() {
        return "SolidPattern{" + "color=" + color + "}";
    }
}
