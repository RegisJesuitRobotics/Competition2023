package frc.robot.utils.led.buffer;

import edu.wpi.first.wpilibj.util.Color8Bit;

public class CombinedAddressableLEDBuffer implements RaiderAddressableLEDBuffer {
    private final RaiderAddressableLEDBuffer first;
    private final RaiderAddressableLEDBuffer second;

    public CombinedAddressableLEDBuffer(RaiderAddressableLEDBuffer first, RaiderAddressableLEDBuffer second) {
        this.first = first;
        this.second = second;
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        if (index < first.getLength()) {
            first.setRGB(index, r, g, b);
        } else {
            second.setRGB(index - first.getLength(), r, g, b);
        }
    }

    @Override
    public int getLength() {
        return first.getLength() + second.getLength();
    }

    @Override
    public Color8Bit getLED8Bit(int index) {
        if (index < first.getLength()) {
            return first.getLED8Bit(index);
        } else {
            return second.getLED8Bit(index - first.getLength());
        }
    }
}
