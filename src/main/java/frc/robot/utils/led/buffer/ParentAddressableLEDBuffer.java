package frc.robot.utils.led.buffer;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class ParentAddressableLEDBuffer extends AddressableLEDBuffer implements RaiderAddressableLEDBuffer {
    /**
     * Constructs a new LED buffer with the specified length.
     *
     * @param length The length of the buffer in pixels
     */
    public ParentAddressableLEDBuffer(int length) {
        super(length);
    }
}
