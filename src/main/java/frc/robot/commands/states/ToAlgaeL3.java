package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Windmill;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToAlgaeL3 extends SequentialCommandGroup {
    public ToAlgaeL3(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, boolean mirror) {
        if (mirror) {
            addCommands(
                    new Windmill(elevatorSubsystem, pivotSubsystem,
                            Constants.Windmill.WindmillState.AlgaePickUpReef3Flip,
                            false));
        } else {
            addCommands(
                    // Switch to a transition state
                    // Maybe turn off all the motors

                    // Check safety
                    new Windmill(elevatorSubsystem, pivotSubsystem, Constants.Windmill.WindmillState.AlgaePickUpReef3,
                            false));
            // Switch to target state.
        }

    }
}