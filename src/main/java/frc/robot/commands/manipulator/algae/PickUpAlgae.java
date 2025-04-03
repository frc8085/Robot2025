package frc.robot.commands.manipulator.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;

// if command is interrupted before algae is picked up, kill command
public class PickUpAlgae extends Command {
        AlgaeSubsystem algaeSubsystem;
        ElevatorSubsystem elevatorSubsystem;
        PivotSubsystem pivotSubsystem;

        public PickUpAlgae(
                        AlgaeSubsystem algaeSubsystem) {

                this.algaeSubsystem = algaeSubsystem;

                addRequirements(algaeSubsystem);

        }

        @Override
        public void initialize() {
                System.out.println("Algae Pickup Started");
        }

        @Override
        public void execute() {
                algaeSubsystem.pickup();
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        algaeSubsystem.stop();
                        System.out.println("Algae PickUp Interrupted");
                } else {
                        Commands.waitSeconds(1);
                        algaeSubsystem.holdAlgae();
                        System.out.println("Algae PickUp Completed");
                }
        }

        @Override
        public boolean isFinished() {
                return algaeSubsystem.isAlgaeDetected();
        }

}
