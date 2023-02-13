package frc.robot.utils.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;
import java.util.TreeMap;

public final class HolonomicTrajectory {
    private final Trajectory trajectory;
    private final RotationSequence rotationSequence;

    public HolonomicTrajectory(Trajectory trajectory, RotationSequence rotationSequence) {
        this.trajectory = trajectory;
        this.rotationSequence = rotationSequence;
    }

    public Trajectory trajectory() {
        return trajectory;
    }

    public RotationSequence rotationSequence() {
        return rotationSequence;
    }

    public State sample(double timestamp) {
        return new State(trajectory.sample(timestamp), rotationSequence.sample(timestamp));
    }

    public HolonomicTrajectory getFlipped() {
        List<Trajectory.State> flippedTrajectoryStates = new ArrayList<>();
        TreeMap<Double, Rotation2d> flippedRotations = new TreeMap<>();

        for (int i = 0; i < trajectory.getStates().size(); i++) {
            Trajectory.State state = trajectory.getStates().get(i);

            flippedTrajectoryStates.add(new Trajectory.State(
                    state.timeSeconds,
                    state.velocityMetersPerSecond,
                    state.accelerationMetersPerSecondSq,
                    new Pose2d(
                            new Translation2d(
                                    state.poseMeters.getTranslation().getX(),
                                    FieldConstants.fieldWidth
                                            - state.poseMeters.getTranslation().getY()),
                            new Rotation2d(
                                    state.poseMeters.getRotation().getCos(),
                                    -state.poseMeters.getRotation().getSin())),
                    -state.curvatureRadPerMeter));
        }

        for (Entry<Double, Rotation2d> entry : rotationSequence.getSequence().entrySet()) {
            flippedRotations.put(
                    entry.getKey(),
                    new Rotation2d(entry.getValue().getCos(), -entry.getValue().getSin()));
        }

        return new HolonomicTrajectory(new Trajectory(flippedTrajectoryStates), new RotationSequence(flippedRotations));
    }

    public record State(Trajectory.State poseState, RotationSequence.State rotationState) {}
}
