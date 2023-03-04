package frc.robot.subsystems.lift;

import static frc.robot.Constants.LiftConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LiftMechanism2d {
    private static final Rotation2d ONE_HUNDRED_EIGHTY_DEGREES = Rotation2d.fromDegrees(180.0);
    private static final double PADDING = Units.inchesToMeters(1.0);
    private static final double MECHANISM2D_WIDTH =
            (HORIZONTAL_BAR_LENGTH * 2) + TOP_HORIZONTAL_TO_BOTTOM_HORIZONTAL.getX() + (PADDING * 2);
    private static final double MECHANISM2D_HEIGHT =
            (HORIZONTAL_BAR_LENGTH * 2) + Math.abs(TOP_HORIZONTAL_TO_BOTTOM_HORIZONTAL.getY()) + (PADDING * 2);

    private final Mechanism2d mechanism2d = new Mechanism2d(MECHANISM2D_WIDTH, MECHANISM2D_HEIGHT);
    private final MechanismLigament2d topHorizontalBarMechanismLigament;
    private final MechanismLigament2d bottomHorizontalBarMechanismLigament;

    public LiftMechanism2d(Color8Bit color) {
        topHorizontalBarMechanismLigament = mechanism2d
                .getRoot(
                        "TopBar",
                        (MECHANISM2D_WIDTH / 2.0) - (TOP_HORIZONTAL_TO_BOTTOM_HORIZONTAL.getX() / 2.0),
                        (MECHANISM2D_HEIGHT / 2.0) - (TOP_HORIZONTAL_TO_BOTTOM_HORIZONTAL.getY() / 2.0))
                .append(new MechanismLigament2d("TopBarBar", HORIZONTAL_BAR_LENGTH, 0.0, 5.0, color));
        bottomHorizontalBarMechanismLigament = mechanism2d
                .getRoot(
                        "BottomBar",
                        (MECHANISM2D_WIDTH / 2.0) + (TOP_HORIZONTAL_TO_BOTTOM_HORIZONTAL.getX() / 2.0),
                        (MECHANISM2D_HEIGHT / 2.0) + (TOP_HORIZONTAL_TO_BOTTOM_HORIZONTAL.getY() / 2.0))
                .append(new MechanismLigament2d("BottomBarBar", HORIZONTAL_BAR_LENGTH, 0.0, 5.0, color));
    }

    public void setAngle(Rotation2d angle) {
        angle = angle.plus(ONE_HUNDRED_EIGHTY_DEGREES).unaryMinus();

        topHorizontalBarMechanismLigament.setAngle(angle);
        bottomHorizontalBarMechanismLigament.setAngle(angle);
    }

    public Mechanism2d getMechanism2dObject() {
        return mechanism2d;
    }
}
