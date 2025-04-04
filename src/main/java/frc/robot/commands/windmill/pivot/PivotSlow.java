package frc.robot.commands.windmill.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Pivot.*;

public class PivotSlow extends Command {
    private final Rotation2d m_targetAngle;
    private final PivotSubsystem m_pivot;
    private final Rotation2d tolerance = PivotArmConstants.kPivotTolerance;

    public PivotSlow(PivotSubsystem pivot, Rotation2d targetAngle) {
        m_targetAngle = targetAngle;
        m_pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        m_pivot.setPosAlt(m_targetAngle);
    }

    @Override
    public boolean isFinished() {
        return m_pivot.isAtTarget(tolerance);
    }
}