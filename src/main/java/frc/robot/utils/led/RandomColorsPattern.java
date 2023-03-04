package frc.robot.utils.led;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.led.buffer.RaiderAddressableLEDBuffer;
import java.util.Random;

public class RandomColorsPattern implements Pattern {
    private final Random random = new Random();
    private final double period;

    public RandomColorsPattern(double period) {
        this.period = period;
    }

    @Override
    public void applyTo(RaiderAddressableLEDBuffer buffer, double patternTime) {
        random.setSeed((long) (patternTime / period));
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.fromHSV((int) (random.nextDouble() * 255), 255, 255));
        }
    }
}
