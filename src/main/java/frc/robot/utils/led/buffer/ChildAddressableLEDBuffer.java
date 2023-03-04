package frc.robot.utils.led.buffer;

import edu.wpi.first.wpilibj.util.Color8Bit;

public class ChildAddressableLEDBuffer implements RaiderAddressableLEDBuffer {
    private final RaiderAddressableLEDBuffer parent;
    private final int startIndex;
    private final int endIndex;

    ChildAddressableLEDBuffer(RaiderAddressableLEDBuffer parent, int startIndex, int endIndex) {
        this.parent = parent;

        this.startIndex = startIndex;
        this.endIndex = endIndex;
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        parent.setRGB(index + startIndex, r, g, b);
    }

    @Override
    public int getLength() {
        return endIndex - startIndex;
    }

    @Override
    public Color8Bit getLED8Bit(int index) {
        return parent.getLED8Bit(index + startIndex);
    }
}
