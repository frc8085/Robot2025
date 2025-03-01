package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.states.ToAlgaeL2;
import frc.robot.commands.states.ToAlgaeL3;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class PickUpAlgaeFromReef extends SequentialCommandGroup {
        public PickUpAlgaeFromReef(
                        AlgaeSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, boolean level, boolean mirror) {

                // level true= L3, false = L2; mirror true = left, false = right
                if (level) {
                        if (mirror) {
                                addCommands(
                                                new ToAlgaeL3(elevatorSubsystem, pivotSubsystem, true),
                                                new PickUpAlgae(algaeSubsystem),
                                                new WaitCommand(.25),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                Constants.Windmill.WindmillState.Home,
                                                                false));
                        } else {
                                addCommands(
                                                new ToAlgaeL3(elevatorSubsystem, pivotSubsystem, false),
                                                new PickUpAlgae(algaeSubsystem),
                                                new WaitCommand(.25),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                Constants.Windmill.WindmillState.Home,
                                                                false));

                        }
                } else {
                        if (mirror) {
                                addCommands(
                                                new ToAlgaeL2(elevatorSubsystem, pivotSubsystem, true),
                                                new PickUpAlgae(algaeSubsystem),
                                                new WaitCommand(.25),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                Constants.Windmill.WindmillState.Home,
                                                                false));

                        } else {
                                addCommands(
                                                new ToAlgaeL2(elevatorSubsystem, pivotSubsystem, false),
                                                new PickUpAlgae(algaeSubsystem));

                        }

                }
        }
}
