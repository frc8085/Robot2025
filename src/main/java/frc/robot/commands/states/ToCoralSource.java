package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Windmill;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToCoralSource extends SequentialCommandGroup {
    public ToCoralSource(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, boolean altButton) {
        if (altButton) {
            addCommands(
                    new Windmill(elevatorSubsystem, pivotSubsystem,
                            Constants.Windmill.WindmillState.CoralPickupAlternate,
                            false));
        } else {
            addCommands(
                    new Windmill(elevatorSubsystem, pivotSubsystem,
                            Constants.Windmill.WindmillState.CoralPickup,
                            false));

        }
    }

}
