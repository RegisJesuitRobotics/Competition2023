package frc.robot.utils.led.buffer;

import edu.wpi.first.wpilibj.util.Color8Bit;

public class ReversedAddressableLEDBuffer extends ChildAddressableLEDBuffer {
    ReversedAddressableLEDBuffer(RaiderAddressableLEDBuffer parent) {
        super(parent, 0, parent.getLength());
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        super.setRGB(getReversedIndex(index), r, g, b);
    }

    @Override
    public Color8Bit getLED8Bit(int index) {
        return super.getLED8Bit(getReversedIndex(index));
    }

    private int getReversedIndex(int index) {
        return getLength() - index - 1;
    }
}
