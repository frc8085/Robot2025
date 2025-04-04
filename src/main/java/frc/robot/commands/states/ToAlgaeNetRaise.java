package frc.robot.commands.states;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.windmill.Windmill;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.Pivot.*;

public class ToAlgaeNetRaise extends SequentialCommandGroup {
    public ToAlgaeNetRaise(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                new PrintCommand("Move to Y Algae Net"),
                new Windmill(elevatorSubsystem, pivotSubsystem,
                        ElevatorConstants.kElevatorNetHeight,
                        Rotation2d.fromDegrees(PivotArmConstants.kPivotAlgaeNetRaise)));

    }

}
