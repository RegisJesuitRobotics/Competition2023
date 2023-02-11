package frc.robot.commands.lift;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftExtensionSuperStructure;

public class PositionClawCommand extends CommandBase {
    private final LiftExtensionSuperStructure superStructure;
    private final Translation2d clawTranslation;

    public PositionClawCommand(Translation2d clawTranslation, LiftExtensionSuperStructure superStructure) {
        this.clawTranslation = clawTranslation;
        this.superStructure = superStructure;

        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        superStructure.setClawPosition(clawTranslation);
    }

    @Override
    public void end(boolean interrupted) {
        superStructure.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return superStructure.getClawPosition().getDistance(clawTranslation) < 0.03;
    }
}
