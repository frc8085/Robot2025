package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveToCoral extends Command {
    private final DriveSubsystem drive;
    private final LimelightSubsystem limelight;

    private final PIDController xPid;
    private final PIDController yPid;
    private final PIDController thetaPid;

    private final double maxSpeed = 0.5;
    private final double maxRotationSpeed = 0.4;

    private final double kP_X = 0.1, kI_X = 0, kD_X = 0;
    private final double kP_Y = 0.07, kI_Y = 0, kD_Y = 0;
    private final double kP_Theta = 0.05, kI_Theta = 0, kD_Theta = 0;

    private final double xTarget = 0; // Align center
    private final double yTarget = 1.5; // Stop 1.5m from tag
    private final double thetaTarget = 0; // Square up to tag

    public DriveToCoral(DriveSubsystem drive, LimelightSubsystem limelight) {
        this.drive = drive;
        this.limelight = limelight;

        xPid = new PIDController(kP_X, kI_X, kD_X);
        yPid = new PIDController(kP_Y, kI_Y, kD_Y);
        thetaPid = new PIDController(kP_Theta, kI_Theta, kD_Theta);

        xPid.setTolerance(0.1);
        yPid.setTolerance(0.5);
        thetaPid.setTolerance(2.0); // Degrees

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        xPid.setSetpoint(xTarget);
        yPid.setSetpoint(yTarget);
        thetaPid.setSetpoint(thetaTarget);
    }

    @Override
    public void execute() {
        if (!limelight.hasTarget("limelight-left")) {
            end(true);
            return;
        }

        // Retrieve AprilTag pose data
        double[] botPose = LimelightHelpers.getBotPose("limelight-left");

        double xError = botPose[0]; // Side-to-side alignment
        double yError = botPose[2]; // Distance to tag (forward movement)
        double thetaError = botPose[5]; // Rotation error (yaw)

        // Compute PID outputs
        double xSpeed = maxSpeed * -xPid.calculate(xError);
        double ySpeed = maxSpeed * -yPid.calculate(yError);
        double rotationSpeed = maxRotationSpeed * -thetaPid.calculate(thetaError);

        // Drive toward the tag while aligning rotation
        drive.drive(xSpeed, ySpeed, rotationSpeed, 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return xPid.atSetpoint() && yPid.atSetpoint() && thetaPid.atSetpoint();
    }
}
