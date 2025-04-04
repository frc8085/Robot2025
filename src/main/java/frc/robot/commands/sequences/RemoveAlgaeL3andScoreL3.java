package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.manipulator.algae.PickUpAlgae;
import frc.robot.commands.windmill.WindmillSlow;
import frc.robot.commands.states.ToAlgaeL3;
import frc.robot.commands.states.ToCoralDropOff3;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import frc.robot.subsystems.Coral.CoralSubsystem;

public class RemoveAlgaeL3andScoreL3 extends SequentialCommandGroup {
        public RemoveAlgaeL3andScoreL3(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
                        AlgaeSubsystem algaeSubsystem, CoralSubsystem coralSubsystem,
                        boolean yellow) {
                addCommands(
                                new PrintCommand("Remove Algae L2 Started"),
                                new ToAlgaeL3(elevatorSubsystem, pivotSubsystem, yellow),
                                new PickUpAlgae(algaeSubsystem),
                                new WaitCommand(.25),
                                new WindmillSlow(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.Home, yellow),
                                new ParallelRaceGroup(new WaitUntilCommand(() -> pivotSubsystem
                                                .pivotAtHomeAngle()), new WaitCommand(.5)),
                                new WindmillSlow(elevatorSubsystem, pivotSubsystem,
                                                Constants.Windmill.WindmillState.CoralDropOff3, yellow),
                                new PrintCommand("Remove Algae L2 Completed"),
                                new ToCoralDropOff3(elevatorSubsystem, pivotSubsystem, yellow),
                                new WaitUntilCommand(elevatorSubsystem::elevatorAtCoralDropOff3Height),
                                new WaitUntilCommand(() -> pivotSubsystem.pivotAtCoralDropOffAngle(yellow)),
                                new WaitCommand(1));

        }
}