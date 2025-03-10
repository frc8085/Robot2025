package frc.robot.commands.movement;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveToReefBlue extends Command {
    DriveSubsystem drive;
    LimelightSubsystem limelight;
    private boolean lostTarget;
    PIDController xPid; // Moves left and right
    PIDController yPid; // Moves forward and back
    double maxSpeed = 1;

    double[] rhat = { 0, 0 }; // Unit vector in the correct direction.

    // TO BE TUNED:
    // double theta = 0; //Angle of the reef.
    double kPX = 0.015;
    double kIX = 0;
    double kDX = 0;
    double kPY = 0.08;
    double kIY = 0;
    double kDY = 0;
    double tolerance = .2;
    double xTarget = -3.40;
    double yTarget = 5.53;

    public DriveToReefBlue(DriveSubsystem drive, LimelightSubsystem limelight) {
        this.drive = drive;
        this.limelight = limelight;

        xPid = new PIDController(kPX, kIX, kDX);
        xPid.setTolerance(tolerance);
        yPid = new PIDController(kPY, kIY, kDY);
        yPid.setTolerance(tolerance);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        yPid.setSetpoint(yTarget);
        lostTarget = false;
        // X setpoint changes with distance, so we update it in execute
    }

    @Override
    public void execute() {
        double tx = limelight.getX("limelight-blue");
        double ty = limelight.getY("limelight-blue");

        xTarget = -2.82 * ty + 16.2 + 8; // Heuristic equation we found
        xPid.setSetpoint(xTarget);

        double xSpeed = maxSpeed * -xPid.calculate(tx);
        double ySpeed = maxSpeed * yPid.calculate(ty);

        // If we got to the correct x, stop moving in that direction.
        if (xPid.atSetpoint()) {
            xSpeed = 0;
        }

        double speed = Math.hypot(xSpeed, ySpeed);

        if (!limelight.hasTarget("limelight-blue")) {
            speed = 0;
            lostTarget = true;
        }

        drive.drive(speed, xSpeed, ySpeed, 0, false);

        SmartDashboard.putNumber("X Error", tx - xPid.getSetpoint());
        SmartDashboard.putNumber("y Error", ty - yPid.getSetpoint());

    }

    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, 0, true);
    }

    public boolean isFinished() {
        return ((xPid.atSetpoint() && yPid.atSetpoint()) || !this.limelight.hasTarget("limelight-blue"));
    }

}