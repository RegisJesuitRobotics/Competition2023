package frc.robot.utils;

import static frc.robot.Constants.LiftConstants.*;
import static frc.robot.Constants.LiftConstants.HORIZONTAL_BAR_LENGTH;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ExtensionConstants;

public class LiftExtensionKinematics {
    private static final Rotation2d NINETY_DEGREES = Rotation2d.fromDegrees(90.0);

    private LiftExtensionKinematics() {}

    public static Pair<Rotation2d, Double> clawPositionToLiftExtensionPosition(Translation2d clawPosition) {
        double relativeHeight = (clawPosition.getY() + HORIZONTAL_BAR_TO_CLAW) - VERTICAL_BAR_HEIGHT_FROM_FLOOR;
        double angleRadians = Math.asin(relativeHeight / HORIZONTAL_BAR_LENGTH);

        double remainingDistance = Math.max(
                clawPosition.getX()
                        - (Math.cos(angleRadians) * HORIZONTAL_BAR_LENGTH)
                        - ExtensionConstants.X_OFFSET_METERS,
                0);

        return Pair.of(Rotation2d.fromRadians(angleRadians), remainingDistance);
    }

    public static Translation2d liftExtensionPositionToClawPosition(Rotation2d liftPosition, double extensionPosition) {
        double x = (extensionPosition + ExtensionConstants.X_OFFSET_METERS)
                + Math.sin(NINETY_DEGREES.plus(liftPosition).getRadians()) * HORIZONTAL_BAR_LENGTH;

        double relativeHeight = Math.cos(liftPosition.getRadians()) * HORIZONTAL_BAR_LENGTH;
        double y = relativeHeight - VERTICAL_BAR_HEIGHT_FROM_FLOOR + HORIZONTAL_BAR_TO_CLAW;

        return new Translation2d(x, y);
    }
}
