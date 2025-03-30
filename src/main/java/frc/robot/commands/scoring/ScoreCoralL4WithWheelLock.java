package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.states.ToCoralDropOff4;
import frc.robot.subsystems.Coral.CoralSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;

public class ScoreCoralL4WithWheelLock extends SequentialCommandGroup {
        public ScoreCoralL4WithWheelLock(
                        ElevatorSubsystem elevatorSubsystem,
                        PivotSubsystem pivotSubsystem, CoralSubsystem coralSubsystem, DriveSubsystem driveSubsystem,
                        boolean yellow) {
                addCommands(
                                new PrintCommand("L4 Coral Score Started"),
                                new PrintCommand("Wheels Locked"),
                                new RunCommand(() -> driveSubsystem.lock()),
                                new ToCoralDropOff4(elevatorSubsystem, pivotSubsystem, yellow),
                                new WaitUntilCommand(elevatorSubsystem::elevatorAtCoralDropOff4Height),
                                new WaitUntilCommand(pivotSubsystem::pivotAtCoral4DropOffAngle),
                                new WaitCommand(.5),
                                new PrintCommand("Coral Eject"),
                                new RunCommand(() -> coralSubsystem.eject(), coralSubsystem).withTimeout(0.5),
                                new InstantCommand(coralSubsystem::stop),
                                new PrintCommand("L4 Coral Score Completed"));
        }
}
