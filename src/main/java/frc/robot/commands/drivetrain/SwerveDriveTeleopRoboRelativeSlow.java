package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.io.Keymap.Controllers;
import frc.robot.subsystems.Drive.*;

public class SwerveDriveTeleopRoboRelativeSlow extends Command {

    DriveSubsystem driveSubsystem;

    public SwerveDriveTeleopRoboRelativeSlow(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Drive Robot Relative Slow");
    }

    @Override
    public void execute() {
        double speedVal = DriveConstants.kSlowDrive;

        double invert = this.driveSubsystem.invertForAlliance();

        double leftX = MathUtil.applyDeadband(-Controllers.driverController.getLeftX(),
                OIConstants.kDriveDeadband);
        double leftY = MathUtil.applyDeadband(-Controllers.driverController.getLeftY(),
                OIConstants.kDriveDeadband);
        double rightX = MathUtil.applyDeadband(-Math.pow(Controllers.driverController.getRightX(), 3),
                OIConstants.kTurnDeadband);

        this.driveSubsystem.drive(
                speedVal,
                leftY, // forward-backward
                leftX, // left-right
                rightX, // rotation
                false);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}