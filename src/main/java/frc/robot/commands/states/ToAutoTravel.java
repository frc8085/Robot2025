package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;

public class ToAutoTravel extends ParallelCommandGroup {
    public ToAutoTravel(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                new Windmill(elevatorSubsystem, pivotSubsystem,
                        Constants.Windmill.WindmillState.AutoTravel, false));
    }
}