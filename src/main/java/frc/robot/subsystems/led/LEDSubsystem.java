package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Robot;
import frc.robot.utils.led.Pattern;
import frc.robot.utils.led.SolidPattern;
import frc.robot.utils.led.buffer.FakeLEDBuffer;
import frc.robot.utils.led.buffer.ParentAddressableLEDBuffer;
import frc.robot.utils.led.buffer.RaiderAddressableLEDBuffer;
import java.util.Collections;
import java.util.List;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED led = new AddressableLED(LEDConstants.PWM_PORT);
    private final ParentAddressableLEDBuffer ledBuffer = new ParentAddressableLEDBuffer(LEDConstants.BACK_LEFT_SIZE
            + LEDConstants.FRONT_LEFT_SIZE
            + LEDConstants.BACK_RIGHT_SIZE
            + LEDConstants.FRONT_RIGHT_SIZE);
    private final RaiderAddressableLEDBuffer[] splitBuffers;
    private final Pattern[] patterns;
    private double patternStartTime = 0.0;

    public LEDSubsystem() {
        splitBuffers = new RaiderAddressableLEDBuffer[4];
        int maxSize = Collections.max(List.of(
                LEDConstants.BACK_LEFT_SIZE,
                LEDConstants.FRONT_LEFT_SIZE,
                LEDConstants.BACK_RIGHT_SIZE,
                LEDConstants.FRONT_RIGHT_SIZE));
        // All buffers are same length and 0 is bottom
        int i = 0;
        splitBuffers[0] = ledBuffer.split(0, LEDConstants.BACK_LEFT_SIZE);
        if (LEDConstants.BACK_LEFT_SIZE < maxSize) {
            splitBuffers[0] = splitBuffers[0].preConcatenate(new FakeLEDBuffer(maxSize - LEDConstants.BACK_LEFT_SIZE));
        }
        i += LEDConstants.BACK_LEFT_SIZE;

        splitBuffers[1] = ledBuffer.split(i, i + LEDConstants.FRONT_LEFT_SIZE);
        splitBuffers[1] = splitBuffers[1].reversed();
        if (LEDConstants.FRONT_LEFT_SIZE < maxSize) {
            splitBuffers[1] = splitBuffers[1].preConcatenate(new FakeLEDBuffer(maxSize - LEDConstants.FRONT_LEFT_SIZE));
        }
        i += LEDConstants.FRONT_LEFT_SIZE;

        splitBuffers[2] = ledBuffer.split(i, i + LEDConstants.BACK_RIGHT_SIZE);
        if (LEDConstants.BACK_RIGHT_SIZE < maxSize) {
            splitBuffers[2] = splitBuffers[2].preConcatenate(new FakeLEDBuffer(maxSize - LEDConstants.BACK_RIGHT_SIZE));
        }
        i += LEDConstants.BACK_RIGHT_SIZE;

        splitBuffers[3] = ledBuffer.split(i, i + LEDConstants.FRONT_RIGHT_SIZE);
        splitBuffers[3] = splitBuffers[3].reversed();
        if (LEDConstants.FRONT_RIGHT_SIZE < maxSize) {
            splitBuffers[3] =
                    splitBuffers[3].preConcatenate(new FakeLEDBuffer(maxSize - LEDConstants.FRONT_RIGHT_SIZE));
        }
        i += LEDConstants.FRONT_RIGHT_SIZE;

        patterns = new Pattern[splitBuffers.length];
        for (int j = 0; j < patterns.length; j++) {
            patterns[j] = new SolidPattern(Color.kBlack);
        }

        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public void setAllPattern(Pattern pattern, boolean forceRestart) {
        setPatterns(forceRestart, pattern, pattern, pattern, pattern);
    }

    public void setAllPattern(Pattern pattern) {
        setAllPattern(pattern, false);
    }

    public void setPatterns(boolean forceRestart, Pattern... desiredPatterns) {
        if (desiredPatterns.length != patterns.length) {
            throw new IllegalArgumentException("Must have " + patterns.length + " patterns");
        }

        boolean anyDifferent = forceRestart;
        for (int i = 0; i < patterns.length; i++) {
            anyDifferent |= patterns[i] != desiredPatterns[i];
            if (desiredPatterns[i] == null) {
                this.patterns[i] = new SolidPattern(Color.kBlack);
            } else {
                this.patterns[i] = desiredPatterns[i];
            }
        }
        if (anyDifferent) {
            patternStartTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void periodic() {
        Robot.startWNode("LEDSubsystem#periodic");
        Robot.startWNode("applyPatterns");
        for (int i = 0; i < splitBuffers.length; i++) {
            patterns[i].applyTo(splitBuffers[i], Timer.getFPGATimestamp() - patternStartTime);
        }
        Robot.endWNode();

        Robot.startWNode("setData");
        led.setData(ledBuffer);
        Robot.endWNode();
        Robot.endWNode();
    }
}
