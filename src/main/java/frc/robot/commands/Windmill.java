package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.Windmill.WindmillState;

public class Windmill extends SequentialCommandGroup {

    private void constructWindmill(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
            double targetHeight,
            Rotation2d targetAngle) {
        // check to see if the elevator and pivot are going to clash with each other
        boolean pivotThroughDangerZone = pivotSubsystem.willPivotThroughDangerZone(targetAngle);
        boolean pivotEndInDangerZone = pivotSubsystem.targetInDangerZone(targetAngle);
        boolean pivotInDangerZone = pivotSubsystem.inDangerZone();

        boolean elevatorEndInDangerZone = elevatorSubsystem.targetInDangerZone(targetHeight);
        boolean elevatorInDangerZone = elevatorSubsystem.inDangerZone();

        if (elevatorEndInDangerZone && pivotEndInDangerZone) {
            // this means that the elevator and pivot are going to clash with each other
            // return nothing
            return;
        }

        // check if we can run the elevator and pivot at the same time
        if (!pivotThroughDangerZone) {
            // we can move the elevator and pivot at the same time
            addCommands(
                    new ParallelCommandGroup(
                            new Elevator(elevatorSubsystem, targetHeight),
                            new Pivot(pivotSubsystem, targetAngle)));
            return;
        }

        // check if the elevator ends in the danger range
        if (elevatorEndInDangerZone) {
            // we can't move the elevator and pivot at the same time
            // we can only move the pivot
            if (elevatorInDangerZone) {
                // cannot move the elevator and pivot at the same time
                // move elevator to safe height, then move pivot
                addCommands(
                        new Elevator(elevatorSubsystem, Constants.ElevatorConstants.kElevatorSafeHeight),
                        new Pivot(pivotSubsystem, targetAngle));
            } else {
                addCommands(
                        new ParallelCommandGroup(
                                new Elevator(elevatorSubsystem, Constants.ElevatorConstants.kElevatorSafeHeight),
                                new Pivot(pivotSubsystem, targetAngle)));
            }
            // then move elevator to target height
            addCommands(
                    new Elevator(elevatorSubsystem, targetHeight));
            return;
        } else {
            // elevator does not end in danger zone, but it starts in the danger zone
            addCommands(
                    new ParallelCommandGroup(
                            new Elevator(elevatorSubsystem, targetHeight),
                            new SequentialCommandGroup(
                                    new WaitUntilCommand(() -> !pivotSubsystem.inDangerZone()),
                                    new Pivot(pivotSubsystem, targetAngle))));
            return;
        }
    }

    public Windmill(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, double targetHeight,
            Rotation2d targetAngle) {
        constructWindmill(elevatorSubsystem, pivotSubsystem, targetHeight, targetAngle);
    }

    public Windmill(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, WindmillState windmillState,
            boolean mirrored) {
        if (mirrored && windmillState.canMirror()) {
            // mirror the windmill pivot arm if possible
        }
        constructWindmill(elevatorSubsystem, pivotSubsystem, windmillState.getElevatorHeight(),
                windmillState.getPivotArmAngle());
    }

}