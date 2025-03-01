package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveToCoral extends Command {
    private final DriveSubsystem drive;
    private final LimelightSubsystem limelight;
    private final PIDController xPid;
    private final PIDController rotPid;

    private final double maxSpeed = 0.5;
    private final double kP = 0.5;
    private final double kI = 0;
    private final double kD = 0;
    private final double tolerance = 0.5;

    private final double rotTarget = 0; // Align rotation to face the AprilTag
    private final double targetArea = 10; // Desired AprilTag size in view

    public DriveToCoral(DriveSubsystem drive, LimelightSubsystem limelight) {
        this.drive = drive;
        this.limelight = limelight;

        xPid = new PIDController(kP, kI, kD);
        xPid.setTolerance(tolerance);

        rotPid = new PIDController(kP, kI, kD);
        rotPid.setTolerance(tolerance);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        xPid.setSetpoint(0); // Center on AprilTag
        rotPid.setSetpoint(rotTarget);
    }

    @Override
    public void execute() {
        if (!limelight.hasTarget("limelight-left")) {
            end(true);
            return;
        }

        double tx = limelight.getX("limelight-left");
        double ta = limelight.getArea("limelight-left");

        double speedX = maxSpeed * -xPid.calculate(tx); // Align horizontally
        double rotation = maxSpeed * -rotPid.calculate(tx); // Rotate to face the tag
        double forwardSpeed = (ta < targetArea) ? 0.3 : 0; // Move forward until close

        drive.drive(maxSpeed, 0, forwardSpeed, 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return xPid.atSetpoint() && limelight.getArea("limelight-left") >= targetArea;
    }
}
