package frc.robot.commands.manipulator.coral;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Coral.*;

// if command is interrupted before coral is picked up, kill command
public class PickUpCoralCurrent extends Command {
        CoralSubsystem coralSubsystem;
        Debouncer debouncer = new Debouncer(.5, Debouncer.DebounceType.kRising);

        public PickUpCoralCurrent(CoralSubsystem coralSubsystem) {

                this.coralSubsystem = coralSubsystem;

                addRequirements(coralSubsystem);

        }

        @Override
        public void initialize() {
                System.out.println("Coral PickUp Started");
        }

        @Override
        public void execute() {
                coralSubsystem.pickup();

        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        coralSubsystem.stop();
                        System.out.println("Coral PickUp Interrupted");
                } else {
                        Commands.waitSeconds(1);
                        coralSubsystem.stop();
                        System.out.println("Coral PickUp Completed");
                }
        }

        @Override
        public boolean isFinished() {
                return (debouncer
                                .calculate(coralSubsystem.getCurrent() >= CoralConstants.kCoralCurrentLimit));
        }

}
