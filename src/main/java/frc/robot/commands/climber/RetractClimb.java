package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class RetractClimb extends Command {
    ClimberSubsystem climberSubsystem;

    public RetractClimb(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);

    }

    @Override
    public void initialize() {
        climberSubsystem.moveDownNoStop();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.climberAtHomePosition();
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }
}
