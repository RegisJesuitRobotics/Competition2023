package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    public enum LEDColorState {
        YELLOW(new Color8Bit(LEDConstants.YELLOW_RGB[0], LEDConstants.YELLOW_RGB[1], LEDConstants.YELLOW_RGB[2])),
        PURPLE(new Color8Bit(LEDConstants.PURPLE_RGB[0], LEDConstants.PURPLE_RGB[1], LEDConstants.PURPLE_RGB[2])),
        OFF(new Color8Bit(0, 0, 0));

        private final Color8Bit color;

        public Color8Bit getColor() {
            return color;
        }

        private LEDColorState(Color8Bit color) {
            this.color = color;
        }
    }

    private final AddressableLED led = new AddressableLED(LEDConstants.PWM_PORT);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.BUFFER_SIZE);

    public LEDSubsystem() {
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public void periodic() {
        // led.setData(ledBuffer);
    }

    public void toggleYellowCone() {
        Color8Bit color;
        if (ledBuffer.getLED8Bit(0).equals(LEDColorState.YELLOW.getColor())) {
            color = LEDColorState.OFF.getColor();
        } else {
            color = LEDColorState.YELLOW.getColor();
        }
        for (var i = 0; i < ledBuffer.getLength(); ++i) {
            ledBuffer.setLED(i, color);
        }
        led.setData(ledBuffer);
    }

    public void togglePurpleCube() {
        Color8Bit color;
        if (ledBuffer.getLED8Bit(0).equals(LEDColorState.PURPLE.getColor())) {
            color = LEDColorState.OFF.getColor();
        } else {
            color = LEDColorState.PURPLE.getColor();
        }
        for (var i = 0; i < ledBuffer.getLength(); ++i) {
            ledBuffer.setLED(i, color);
        }
        led.setData(ledBuffer);
    }
}
