package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED led = new AddressableLED(LEDConstants.PWM_PORT);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.BUFFER_SIZE);
    // private final int targetNumLoops = 3;
    // private int streakLED = 0;
    // private int numLoopsSincePeriodic = 0;
    private boolean colorSet = false;

    public LEDSubsystem() {
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        setRaiderIdleColors();
        led.start();
    }

    public void periodic() {
        // led.setData(ledBuffer);
    }

    public void toggleYellowCone() {
        if (!ledBuffer.getLED(0).equals(Color.kGold)) {
            setSolidColor(Color.kGold);
            // colorSet = true;
        } else {
            setRaiderIdleColors();
            // colorSet = false;
        }
    }

    public void togglePurpleCube() {
        if (!ledBuffer.getLED(0).equals(Color.kDarkViolet)) {
            setSolidColor(Color.kDarkViolet);
            // colorSet = true;
        } else {
            setRaiderIdleColors();
            // colorSet = false;
        }
    }

    private void setRaiderIdleColors() {
        // setStreak(Color.kDarkRed, Color.kGray);
        for (int i = 0; i < ledBuffer.getLength(); i+=2) {
            ledBuffer.setLED(i, Color.kDarkRed);
        }
        for (int i = 1; i < ledBuffer.getLength(); i+=2) {
            ledBuffer.setLED(i, Color.kWhite);
        }
        led.setData(ledBuffer);
    }

    private void setSolidColor(Color color) {
        for (int i = 0; i < ledBuffer.getLength(); ++i) {
            ledBuffer.setLED(i, color);
        }
        led.setData(ledBuffer);
    }

/*
    private void setStreak(Color primary, Color secondary) {
        setSolidColor(primary);
        ledBuffer.setLED(streakLED, secondary);
        ledBuffer.setLED((streakLED + 1) % ledBuffer.getLength(), secondary);
        ledBuffer.setLED((streakLED + 2) % ledBuffer.getLength(), secondary);

        if (numLoopsSincePeriodic == 0) {
            ++streakLED;
            streakLED %= ledBuffer.getLength();
        }

        led.setData(ledBuffer);

        ++numLoopsSincePeriodic;
        numLoopsSincePeriodic %= targetNumLoops;
    }
*/

}
