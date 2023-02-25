package frc.robot.commands.lift;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.lift.LiftSubsystem;

public class SetLiftPositionCommand extends CommandBase {
    private final LiftSubsystem liftSubsystem;
    private final Rotation2d desiredPosition;

    public SetLiftPositionCommand(Rotation2d desiredPosition, LiftSubsystem liftSubsystem) {
        this.desiredPosition = desiredPosition;
        this.liftSubsystem = liftSubsystem;

        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        liftSubsystem.setDesiredArmAngle(desiredPosition);
    }

    @Override
    public void end(boolean interrupted) {
        liftSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return liftSubsystem.atClosedLoopGoal();
    }
}
