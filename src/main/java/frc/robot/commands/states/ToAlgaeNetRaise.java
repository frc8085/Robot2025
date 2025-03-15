package frc.robot.commands.states;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotArmConstants;
import frc.robot.commands.windmill.Windmill;
import frc.robot.commands.windmill.WindmillAlgaeNet;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ToAlgaeNetRaise extends SequentialCommandGroup {
    public ToAlgaeNetRaise(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                new PrintCommand("Move to Y Algae Net"),
                new WindmillAlgaeNet(elevatorSubsystem, pivotSubsystem,
                        ElevatorConstants.kElevatorNetHeight,
                        Rotation2d.fromDegrees(PivotArmConstants.kPivotAlgaeNetRaise)));

    }

}
