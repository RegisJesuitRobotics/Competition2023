package frc.robot.commands.drive.characterize.modes;

public class DynamicCharacterizationMode implements CharacterizationMode {
    private final double voltage;

    public DynamicCharacterizationMode(double voltage) {
        this.voltage = voltage;
    }

    @Override
    public double getVoltage(double time) {
        return voltage;
    }

    @Override
    public String getTestMetaData() {
        return "fast-" + (voltage > 0.0 ? "forward" : "backward");
    }
}
