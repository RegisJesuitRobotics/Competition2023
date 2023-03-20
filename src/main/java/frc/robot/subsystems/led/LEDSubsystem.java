package frc.robot.subsystems.led;

import static frc.robot.Constants.LEDConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.led.Pattern;
import frc.robot.utils.led.SolidPattern;
import frc.robot.utils.led.buffer.FakeLEDBuffer;
import frc.robot.utils.led.buffer.ParentAddressableLEDBuffer;
import frc.robot.utils.led.buffer.RaiderAddressableLEDBuffer;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED led = new AddressableLED(PWM_PORT);
    private final ParentAddressableLEDBuffer ledBuffer = new ParentAddressableLEDBuffer(TOTAL_SIZE);
    private final RaiderAddressableLEDBuffer[] splitBuffers;
    private final Pattern[] patterns;
    private double patternStartTime = 0.0;

    public LEDSubsystem() {
        splitBuffers = new RaiderAddressableLEDBuffer[4];
        // All buffers are same length and 0 is bottom
        int i = 0;
        splitBuffers[0] = ledBuffer.split(0, BACK_LEFT_SIZE);
        if (BACK_LEFT_SIZE < MAX_SIZE) {
            splitBuffers[0] = splitBuffers[0].preConcatenate(new FakeLEDBuffer(MAX_SIZE - BACK_LEFT_SIZE));
        }
        i += BACK_LEFT_SIZE;

        splitBuffers[1] = ledBuffer.split(i, i + FRONT_LEFT_SIZE);
        splitBuffers[1] = splitBuffers[1].reversed();
        if (FRONT_LEFT_SIZE < MAX_SIZE) {
            splitBuffers[1] = splitBuffers[1].preConcatenate(new FakeLEDBuffer(MAX_SIZE - FRONT_LEFT_SIZE));
        }
        i += FRONT_LEFT_SIZE;

        splitBuffers[2] = ledBuffer.split(i, i + BACK_RIGHT_SIZE);
        if (BACK_RIGHT_SIZE < MAX_SIZE) {
            splitBuffers[2] = splitBuffers[2].preConcatenate(new FakeLEDBuffer(MAX_SIZE - BACK_RIGHT_SIZE));
        }
        i += BACK_RIGHT_SIZE;

        splitBuffers[3] = ledBuffer.split(i, i + FRONT_RIGHT_SIZE);
        splitBuffers[3] = splitBuffers[3].reversed();
        if (FRONT_RIGHT_SIZE < MAX_SIZE) {
            splitBuffers[3] = splitBuffers[3].preConcatenate(new FakeLEDBuffer(MAX_SIZE - FRONT_RIGHT_SIZE));
        }
        i += FRONT_RIGHT_SIZE;

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
