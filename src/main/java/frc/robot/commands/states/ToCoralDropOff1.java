package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Windmill;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToCoralDropOff1 extends SequentialCommandGroup {
    public ToCoralDropOff1(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, boolean yellow) {
        if (yellow) {
            addCommands(
                    new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.CoralDropOff1,
                            true));
        } else {
            addCommands(
                    // Switch to a transition state
                    // Maybe turn off all the motors

                    // Check safety
                    new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.CoralDropOff1,
                            false));
            // Switch to target state.
        }

    }
}