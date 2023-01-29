package frc.robot.utils.trajectory;

import edu.wpi.first.math.trajectory.Trajectory;

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

    public record State(Trajectory.State poseState, RotationSequence.State rotationState) {}
}
