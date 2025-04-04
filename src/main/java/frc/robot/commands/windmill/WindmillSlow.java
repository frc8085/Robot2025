package frc.robot.commands.windmill;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.Pivot.*;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.windmill.pivot.*;
import frc.robot.commands.windmill.elevator.*;
import frc.robot.Constants.TuningModeConstants;
import frc.robot.Constants.Windmill.WindmillState;

public class WindmillSlow extends Command {

        double targetHeight;
        Rotation2d targetAngle;
        ElevatorSubsystem elevatorSubsystem;
        PivotSubsystem pivotSubsystem;

        boolean finished = false;

        public WindmillSlow(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem, double targetHeight,
                        Rotation2d targetAngle) {

                this.elevatorSubsystem = elevatorSubsystem;
                this.pivotSubsystem = pivotSubsystem;
                this.targetAngle = targetAngle;
                this.targetHeight = targetHeight;
        }

        // if the same elevator/arm position can be used on both sides of the robot,
        // then set the pivot arm angle as the mirror when on the right (intake) side
        public WindmillSlow(ElevatorSubsystem elevatorSubsystem, PivotSubsystem pivotSubsystem,
                        WindmillState windmillState,
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
                        } else if (!pivotWillSwingThrough) {
                                commands.addCommands(
                                                new PrintCommand("Windmill: Pivot will not swing through danger zone"));
                        } else if (!elevatorInDangerZone && !elevatorEndInDangerZone) {
                                commands.addCommands(new PrintCommand("Windmill: Elevator will not be in danger zone"));
                        }
                }

                // if pivot will not swing through the danger zone, then move the elevator and
                // pivot together

                if (!pivotWillSwingThrough) {
                        commands.addCommands(
                                        new ParallelCommandGroup(
                                                        new Elevator(elevatorSubsystem, targetHeight),
                                                        new PivotSlow(pivotSubsystem, targetAngle)));

                }
                // if elevator starts and ends outside the danger zone, move the elevator and
                // pivot together
                else if (!elevatorInDangerZone && !elevatorEndInDangerZone) {
                        commands.addCommands(
                                        new ParallelCommandGroup(
                                                        new Elevator(elevatorSubsystem, targetHeight),
                                                        new PivotSlow(pivotSubsystem, targetAngle)));
                }
                // if elevator starts outside the danger zone and ends in the danger zone, then
                // turn the pivot and lower the elevator to the safe height and then finish
                // lowering the elevator once the pivot is clear
                else if (!elevatorInDangerZone && elevatorEndInDangerZone) {
                        commands.addCommands(
                                        new ParallelCommandGroup(
                                                        new PivotSlow(pivotSubsystem, targetAngle),
                                                        new Elevator(elevatorSubsystem,
                                                                        ElevatorConstants.kElevatorSafeHeightMax)),
                                        new WaitUntilCommand(() -> !pivotSubsystem.inDangerZone()),
                                        new Elevator(elevatorSubsystem, targetHeight));

                } // if elevator starts in the danger zone and ends in the danger zone, then raise
                  // the elevator to the safe height, turn the pivot, then lower the elevator to
                  // the final height

                else if (elevatorInDangerZone && elevatorEndInDangerZone) {
                        commands.addCommands(
                                        new SequentialCommandGroup(
                                                        new Elevator(elevatorSubsystem,
                                                                        ElevatorConstants.kElevatorSafeHeightMax),
                                                        new PivotSlow(pivotSubsystem, targetAngle),
                                                        new Elevator(elevatorSubsystem, targetHeight)));
                }

                // if elevator starts in danger zone and does not end in danger zone, then move
                // the elevator to the target
                // height, and once the elevator reaches the safe height, move the pivot arm

                else {
                        commands.addCommands(
                                        new ParallelCommandGroup(
                                                        new Elevator(elevatorSubsystem, targetHeight),
                                                        new SequentialCommandGroup(
                                                                        new WaitUntilCommand(
                                                                                        elevatorSubsystem::isSafeToPivot),
                                                                        new PivotSlow(pivotSubsystem, targetAngle))));

                }

                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(commands
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