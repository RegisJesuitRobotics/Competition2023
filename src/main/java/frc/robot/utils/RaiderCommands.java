package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.*;
import java.util.function.BooleanSupplier;

public class RaiderCommands {
    private RaiderCommands() {}

    public static ConditionalCommandBuilder ifCondition(BooleanSupplier condition) {
        return new ConditionalCommandBuilder(condition);
    }

    public static CommandBase startNoEnd(Runnable start, Subsystem... requirements) {
        return new StartEndCommand(start, () -> {}, requirements);
    }

    public static class ConditionalCommandBuilder {
        private final BooleanSupplier condition;
        private Command ifTrue;

        private ConditionalCommandBuilder(BooleanSupplier condition) {
            this.condition = condition;
        }

        public ConditionalCommandBuilder then(Command command) {
            ifTrue = command;
            return this;
        }

        public ConditionalCommand otherwise(Command command) {
            return new ConditionalCommand(ifTrue, command, condition);
        }
    }
}
