package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToAlgaeNetRightCommand extends SequentialCommandGroup {
    public ToAlgaeNetRightCommand(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                // Switch to a transition state
                // Maybe turn off all the motors

                // Check safety
                new ElevatorToSafe(elevatorSubsystem),
                // Move pivot
                new PivotToAlgaeNetRight(pivotSubsystem),
                // Move elevator
                new ElevatorToAlgaeNetRight(elevatorSubsystem));
        // Switch to target state.
    }
}