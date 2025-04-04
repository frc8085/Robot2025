package frc.robot.commands.autoCommands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.SwerveDriveChoreoFollow;
import frc.robot.commands.sequences.RemoveAlgaeL3andScoreL3;
import frc.robot.commands.windmill.Windmill;
import frc.robot.commands.windmill.elevator.ZeroElevator;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Coral.CoralSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;

public class ChoreoAutoOppoToL4 extends SequentialCommandGroup {
        public ChoreoAutoOppoToL4(DriveSubsystem driveSubsystem, AlgaeSubsystem algaeSubsystem,
                        ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
                        CoralSubsystem coralSubsystem) {

                Optional<Trajectory<SwerveSample>> path1 = Choreo.loadTrajectory("OppoBargeToReef22");
                Optional<Trajectory<SwerveSample>> path2 = Choreo.loadTrajectory("Reef22ToSource");
                Optional<Trajectory<SwerveSample>> path3 = Choreo.loadTrajectory("SourceToReef17L");
                Optional<Trajectory<SwerveSample>> path4 = Choreo.loadTrajectory("Reef17LToSource");
                Optional<Trajectory<SwerveSample>> path5 = Choreo.loadTrajectory("SourceToReef17R");

                addCommands(
                                new SequentialCommandGroup(
                                                new ZeroElevator(elevatorSubsystem),
                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                Constants.Windmill.WindmillState.Home,
                                                                false)),
                                new SwerveDriveChoreoFollow(
                                                driveSubsystem,
                                                path1, true),
                                new RemoveAlgaeL3andScoreL3(elevatorSubsystem, pivotSubsystem, algaeSubsystem,
                                                coralSubsystem, false),
                                new RunCommand(() -> coralSubsystem.eject(), coralSubsystem).withTimeout(1),
                                new InstantCommand(coralSubsystem::stop),

                                new AutoToHomeCommand(elevatorSubsystem, pivotSubsystem),
                                new AutoWaitUntilElevatorBelowSafeTravelHeight(elevatorSubsystem),
                                new ParallelDeadlineGroup(
                                                new AutoCoralPickup(elevatorSubsystem, pivotSubsystem, coralSubsystem),
                                                new SwerveDriveChoreoFollow(
                                                                driveSubsystem,
                                                                path2, false)),
                                new SwerveDriveChoreoFollow(
                                                driveSubsystem,
                                                path3, false),
                                new AutoScoreCoralL4(algaeSubsystem, elevatorSubsystem, pivotSubsystem, coralSubsystem,
                                                driveSubsystem,
                                                isScheduled()),
                                new SwerveDriveChoreoFollow(
                                                driveSubsystem,
                                                path4, false),
                                new SwerveDriveChoreoFollow(
                                                driveSubsystem,
                                                path5, false),
                                new AutoScoreCoralL4(algaeSubsystem, elevatorSubsystem, pivotSubsystem, coralSubsystem,
                                                driveSubsystem,
                                                isScheduled()));
        }
}