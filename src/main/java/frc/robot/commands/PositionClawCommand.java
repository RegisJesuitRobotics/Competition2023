package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.utils.LiftExtensionKinematics;

// TODO: Make this parallel command group
public class PositionClawCommand extends CommandBase {
    private final LiftSubsystem liftSubsystem;
    private final ExtensionSubsystem extensionSubsystem;
    private final Translation2d clawTranslation;

    public PositionClawCommand(
            Translation2d clawTranslation, LiftSubsystem liftSubsystem, ExtensionSubsystem extensionSubsystem) {
        this.clawTranslation = clawTranslation;
        this.liftSubsystem = liftSubsystem;
        this.extensionSubsystem = extensionSubsystem;

        addRequirements(liftSubsystem, extensionSubsystem);
    }

    @Override
    public void initialize() {
        Pair<Rotation2d, Double> desiredPositions =
                LiftExtensionKinematics.clawPositionToLiftExtensionPosition(clawTranslation);
        liftSubsystem.setArmAngle(desiredPositions.getFirst());
        extensionSubsystem.setDistance(desiredPositions.getSecond());
    }

    @Override
    public void end(boolean interrupted) {
        liftSubsystem.stopMovement();
        extensionSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return LiftExtensionKinematics.liftExtensionPositionToClawPosition(
                                liftSubsystem.getArmAngle(), extensionSubsystem.getPosition())
                        .getDistance(clawTranslation)
                < 0.03;
    }
}
