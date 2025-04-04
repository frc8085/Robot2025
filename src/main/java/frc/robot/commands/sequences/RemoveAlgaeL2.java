package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.manipulator.algae.PickUpAlgae;
import frc.robot.commands.windmill.Windmill;
import frc.robot.commands.windmill.WindmillSlow;
import frc.robot.commands.states.ToAlgaeL2;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;

public class RemoveAlgaeL2 extends SequentialCommandGroup {
        public RemoveAlgaeL2(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
                        AlgaeSubsystem algaeSubsystem,
                        boolean yellow) {
                addCommands(
                                new PrintCommand("Remove Algae L2 Started"),
                                new ToAlgaeL2(elevatorSubsystem, pivotSubsystem, yellow),
                                new PickUpAlgae(algaeSubsystem),
                                new WaitCommand(.25),
                                new WindmillSlow(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.Home, yellow),
                                new ParallelRaceGroup(new WaitUntilCommand(() -> pivotSubsystem
                                                .pivotAtHomeAngle()), new WaitCommand(.5)),
                                new WindmillSlow(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.CoralDropOff3, yellow),
                                new PrintCommand("Remove Algae L2 Completed"));

        }
}