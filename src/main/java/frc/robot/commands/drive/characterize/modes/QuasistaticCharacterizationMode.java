package frc.robot.commands.drive.characterize.modes;

public class QuasistaticCharacterizationMode implements CharacterizationMode {
    private final double voltsPerSecond;

    public QuasistaticCharacterizationMode(double voltsPerSecond) {
        this.voltsPerSecond = voltsPerSecond;
    }

    @Override
    public double getVoltage(double time) {
        return voltsPerSecond * time;
    }
}
