package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.SwerveDriveChoreoFollow;
import frc.robot.commands.scoring.ScoreAlgaeNetNoTurn;
import frc.robot.commands.sequences.RemoveAlgaeL2noCoral;
import frc.robot.commands.windmill.InitializePivotAndElevator;
import frc.robot.commands.windmill.Windmill;
import frc.robot.subsystems.Algae.AlgaeSubsystem;
import frc.robot.subsystems.Coral.CoralSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Pivot.PivotSubsystem;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import java.util.Optional;

public class ChoreoAutoCenterBargeAbv extends SequentialCommandGroup {
        public ChoreoAutoCenterBargeAbv(DriveSubsystem driveSubsystem, PivotSubsystem pivotSubsystem,
                        ElevatorSubsystem elevatorSubsystem, AlgaeSubsystem algaeSubsystem,
                        CoralSubsystem coralSubsystem) {

                Optional<Trajectory<SwerveSample>> path1 = Choreo.loadTrajectory("CenterBargeTest");
                // Optional<Trajectory<SwerveSample>> path2 =
                // Choreo.loadTrajectory("Reef21ToScoreBarge");
                // Optional<Trajectory<SwerveSample>> path3 =
                // Choreo.loadTrajectory("ScoreBargeToReef20");
                // Optional<Trajectory<SwerveSample>> path4 =
                // Choreo.loadTrajectory("Reef20ToScoreBarge");
                // Optional<Trajectory<SwerveSample>> path5 =
                // Choreo.loadTrajectory("ScoreBargeToSource");

                addCommands(
                                new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                new InitializePivotAndElevator(pivotSubsystem,
                                                                                elevatorSubsystem),
                                                                new Windmill(elevatorSubsystem, pivotSubsystem,
                                                                                Constants.Windmill.WindmillState.Home,
                                                                                false)),
                                                new SwerveDriveChoreoFollow(
                                                                driveSubsystem,
                                                                path1, true)),
                                // new RemoveAlgaeL2noCoral(elevatorSubsystem, pivotSubsystem,
                                // algaeSubsystem,
                                // false),
                                new WaitCommand(1));
        }
}