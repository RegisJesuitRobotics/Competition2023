package frc.robot.subsystems;

import static frc.robot.Constants.LiftConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;

public class LiftExtensionSuperStructure extends SubsystemBase {
    private final LiftSubsystem liftSubsystem = new LiftSubsystem();
    private final ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem();

    public void setClawPosition(Translation2d clawPosition) {
        double relativeHeight = VERTICAL_BAR_HEIGHT_FROM_FLOOR - (clawPosition.getY() + HORIZONTAL_BAR_TO_CLAW);
        double angleRadians = Math.acos(relativeHeight / HORIZONTAL_BAR_LENGTH);
        liftSubsystem.setArmAngle(Rotation2d.fromRadians(angleRadians));

        double remainingDistance = Math.max(clawPosition.getX() - (Math.sin(angleRadians) * HORIZONTAL_BAR_LENGTH), 0);
        extensionSubsystem.setDistance(remainingDistance);
    }

    public void stopMovement() {
        liftSubsystem.stopMovement();
        extensionSubsystem.stopMovement();
    }

    public void setLiftVoltage(double voltage) {
        liftSubsystem.setVoltage(voltage);
    }

    public Translation2d getClawPosition() {
        double liftAngleRadians = liftSubsystem.getArmAngle().getRadians();

        double x = extensionSubsystem.getDistance() + Math.sin(liftAngleRadians) * HORIZONTAL_BAR_LENGTH;

        double relativeHeight = Math.cos(liftAngleRadians) * HORIZONTAL_BAR_LENGTH;
        double y = relativeHeight - VERTICAL_BAR_HEIGHT_FROM_FLOOR + HORIZONTAL_BAR_TO_CLAW;

        return new Translation2d(x, y);
    }
}
