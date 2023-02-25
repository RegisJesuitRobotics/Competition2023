package frc.robot.utils.geometry;

import edu.wpi.first.math.geometry.Translation2d;

public class Rectangle {
    private final Translation2d bottomCorner;
    private final Translation2d topCorner;

    public Rectangle(Translation2d bottomCorner, Translation2d topCorner) {
        this.bottomCorner = bottomCorner;
        this.topCorner = topCorner;
    }

    public boolean isPointInside(Translation2d translation2d) {
        return translation2d.getX() >= bottomCorner.getX()
                && translation2d.getX() <= topCorner.getX()
                && translation2d.getY() >= bottomCorner.getY()
                && translation2d.getY() <= topCorner.getY();
    }
}
