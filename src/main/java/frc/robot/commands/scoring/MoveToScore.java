package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
// import frc.robot.commands.movement.AutoDriveMeters;
import frc.robot.subsystems.DriveSubsystem;

public class MoveToScore extends Command {
    DriveSubsystem drive;

    public MoveToScore(DriveSubsystem drive) {
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        switch (RobotContainer.scoreDirection) {
            case LEFT:
                // Move to the left
                // new AutoDriveMeters(drive, 0, 0.2);
                break;
            case RIGHT:
                // Move to the right
                // new AutoDriveMeters(drive, 0, -0.2);

                break;
            case UNDECIDED:
                // Do nothing until left or right is decided.
                break;
        }
    }
}