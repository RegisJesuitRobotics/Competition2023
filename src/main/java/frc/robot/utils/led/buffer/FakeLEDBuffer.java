package frc.robot.utils.led.buffer;

import edu.wpi.first.wpilibj.util.Color8Bit;

public class FakeLEDBuffer implements RaiderAddressableLEDBuffer {
    private final byte[] buffer;

    public FakeLEDBuffer(int length) {
        buffer = new byte[length * 4];
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
        buffer[index * 4] = (byte) b;
        buffer[(index * 4) + 1] = (byte) g;
        buffer[(index * 4) + 2] = (byte) r;
        buffer[(index * 4) + 3] = 0;
    }

    @Override
    public int getLength() {
        return buffer.length / 4;
    }

    @Override
    public Color8Bit getLED8Bit(int index) {
        return new Color8Bit(buffer[index * 4 + 2] & 0xFF, buffer[index * 4 + 1] & 0xFF, buffer[index * 4] & 0xFF);
    }
}
