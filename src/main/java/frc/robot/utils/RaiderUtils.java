package frc.robot.utils;

import com.ctre.phoenix.ErrorCode;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.revrobotics.REVLibError;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import java.util.ArrayList;
import java.util.List;

public class RaiderUtils {
    public static int getSolenoidValueToInt(Value value) {
        return switch (value) {
            case kOff -> 0;
            case kForward -> 1;
            case kReverse -> 2;
        };
    }

    public static boolean anyTrue(boolean[] array) {
        for (boolean bool : array) {
            if (bool) {
                return true;
            }
        }
        return false;
    }

    /**
     * @param code the error code
     * @return true if there was a fault
     */
    public static boolean checkCTREError(ErrorCode code) {
        return code != ErrorCode.OK;
    }


    public static boolean checkRevError(REVLibError code) {
        return code != REVLibError.kOk;
    }

    public static boolean shouldFlip() {
        return DriverStation.getAlliance() == Alliance.Red;
    }

    public static Pose2d flipIfShould(Pose2d pose) {
        if (shouldFlip()) {
            return allianceFlip(pose);
        }
        return pose;
    }

    public static Rotation2d flipIfShould(Rotation2d rotation2d) {
        if (shouldFlip()) {
            return allianceFlip(rotation2d);
        }
        return rotation2d;
    }

    public static Rotation2d allianceFlip(Rotation2d rotation2d) {
        return new Rotation2d(-rotation2d.getCos(), rotation2d.getSin());
    }

    public static Pose2d allianceFlip(Pose2d pose) {
        return new Pose2d(
                new Translation2d(
                        FieldConstants.fieldLength - pose.getTranslation().getX(),
                        pose.getTranslation().getY()),
                allianceFlip(pose.getRotation()));
    }

    public static PathPlannerTrajectory allianceFlip(PathPlannerTrajectory trajectory) {
        List<Trajectory.State> newStates = new ArrayList<>();

        for (State s : trajectory.getStates()) {
            PathPlannerState state = (PathPlannerState) s;

            newStates.add(allianceFlip(state));
        }

        return new PathPlannerTrajectory(
                newStates,
                trajectory.getMarkers(),
                trajectory.getStartStopEvent(),
                trajectory.getEndStopEvent(),
                trajectory.fromGUI);
    }

    public static PathPlannerState allianceFlip(PathPlannerState state) {
        Pose2d transformedPose = allianceFlip(state.poseMeters);
        Rotation2d transformedHolonomicRotation = allianceFlip(state.holonomicRotation);

        // Some things are private so just have this do those for us, then we will overwrite what needs changing
        PathPlannerState transformedState = PathPlannerTrajectory.transformStateForAlliance(state, Alliance.Red);
        transformedState.poseMeters = transformedPose;
        transformedState.holonomicRotation = transformedHolonomicRotation;

        return transformedState;
    }
}
