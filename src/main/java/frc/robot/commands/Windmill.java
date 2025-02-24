package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.TuningModeConstants;
import frc.robot.Constants.Windmill.WindmillState;

public class Windmill extends Command {

        double targetHeight;
        Rotation2d targetAngle;
        ElevatorSubsystem elevatorSubsystem;
        PivotSubsystem pivotSubsystem;

        boolean finished = false;

        public Windmill(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, double targetHeight,
                        Rotation2d targetAngle) {

                this.elevatorSubsystem = elevatorSubsystem;
                this.pivotSubsystem = pivotSubsystem;
                this.targetAngle = targetAngle;
                this.targetHeight = targetHeight;
        }

        // if the same elevator/arm position can be used on both sides of the robot,
        // then set the pivot arm angle as the mirror when on the right (intake) side
        public Windmill(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, WindmillState windmillState,
                        boolean mirrored) {
                var rotation_target = windmillState.getPivotArmAngle();
                if (mirrored && windmillState.canMirror()) {
                        // mirror the windmill pivot arm if possible (flip around the -90 degree angle)
                        rotation_target = Rotation2d.fromDegrees(-rotation_target.getDegrees());
                }
                this.elevatorSubsystem = elevatorSubsystem;
                this.pivotSubsystem = pivotSubsystem;
                this.targetAngle = rotation_target;
                this.targetHeight = windmillState.getElevatorHeight();
        }

        // public void finish() {
        // this.finished = true;
        // }

        @Override
        public void initialize() {

                // check if elevator is currently below the safe pivot arm movement height (i.e.
                // danger zone)
                boolean elevatorInDangerZone = elevatorSubsystem.inDangerZone();
                // check if end height of the elevator is in the danger zone
                boolean elevatorEndInDangerZone = elevatorSubsystem.targetInDangerZone(this.targetHeight);
                // check if the arm will go through the danger zone when below the safe height
                boolean pivotWillSwingThrough = pivotSubsystem.willPivotThroughDangerZone(this.targetAngle);

                SequentialCommandGroup commands = new SequentialCommandGroup();

                // logging
                if (TuningModeConstants.kElevatorTuning) {

                        if (elevatorInDangerZone && pivotWillSwingThrough) {
                                commands.addCommands(
                                                new PrintCommand("Windmill: Pivot will swing through danger zone"));
                        } else {
                                commands.addCommands(
                                                new PrintCommand("Windmill: Pivot will not swing through danger zone"));
                        }
                }
                // if elevator is not in danger zone or pivot will not swing through the danger
                // zone, move the elevator and pivot arm in parallel

                if (!elevatorInDangerZone || !pivotWillSwingThrough) {
                        commands.addCommands(
                                        new ParallelCommandGroup(
                                                        new Elevator(elevatorSubsystem, targetHeight),
                                                        new Pivot(pivotSubsystem, targetAngle)));

                }
                // if elevator ends in the danger zone, then move the elevator to the safe
                // height, move the pivot, then continue to the final height
                else {
                        if (elevatorEndInDangerZone) {
                                commands.addCommands(
                                                new SequentialCommandGroup(
                                                                new Elevator(elevatorSubsystem,
                                                                                Constants.ElevatorConstants.kElevatorSafeHeightMax),
                                                                new Pivot(pivotSubsystem, targetAngle),
                                                                new Elevator(elevatorSubsystem, targetHeight)));
                        }

                        // if elevator does not end in danger zone, then move the elevator to the target
                        // height, and once the elevator reaches the safe height, move the pivot arm

                        else {
                                commands.addCommands(
                                                new ParallelCommandGroup(
                                                                new Elevator(elevatorSubsystem, targetHeight),
                                                                new SequentialCommandGroup(
                                                                                new WaitUntilCommand(
                                                                                                () -> !elevatorSubsystem
                                                                                                                .inDangerZone()),
                                                                                new Pivot(pivotSubsystem,
                                                                                                targetAngle))));

                        }
                }

                CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                                commands
                                // new InstantCommand(() -> this.finish())
                                ));
        }

        @Override
        public boolean isFinished() {
                return true;
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        this.finished = true;
                }
        }
}