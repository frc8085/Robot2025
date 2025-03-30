package frc.robot.commands.manipulator.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.states.ToHomeCommand;
import frc.robot.commands.windmill.pivot.Pivot;
import frc.robot.subsystems.Coral.CoralSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Pivot.PivotArmConstants;
import frc.robot.subsystems.Pivot.PivotSubsystem;

public class EjectL4Coral extends SequentialCommandGroup {
        public EjectL4Coral(
                        CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem) {
                addCommands(
                                new PrintCommand("Coral Eject Started"),
                                new RunCommand(() -> coralSubsystem.eject(), coralSubsystem).withTimeout(.5),
                                new InstantCommand(coralSubsystem::stop),
                                new Pivot(pivotSubsystem,
                                                Rotation2d.fromDegrees(PivotArmConstants.kPivotCoralDropOff4 - 20)),
                                new ToHomeCommand(elevatorSubsystem, pivotSubsystem));

        }
}
