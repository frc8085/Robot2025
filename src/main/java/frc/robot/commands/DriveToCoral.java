package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class DriveToCoral extends Command {
    DriveSubsystem drive;
    LimelightSubsystem limelight;
    PIDController xPid;
    double maxSpeed = 0.5;

    double[] rhat = { 0, 0 }; // Unit vector in the correct direction.

    // TO BE TUNED:
    // double theta = 0; //Angle of the reef.
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double tolerance = 0.01;
    double xTarget = -17;
    double yTarget = 7.46;

    public DriveToCoral(DriveSubsystem drive, LimelightSubsystem limelight) {
        this.drive = drive;
        this.limelight = limelight;

        xPid = new PIDController(kP, kI, kD);
        xPid.setTolerance(tolerance);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        xPid.setSetpoint(xTarget);

        // int tagId = limelight.getID("limelight-left"); //In the future, you can add
        // some logic to switch b/w left & right
        // switch(tagId){ //TODO: Finish the rest with the specific unit vectors.
        // case 6: //Assuming that there is a explicit positive direction that doesn't
        // change red vs. blue
        // case 22:
        // rhat[0] = Math.cos(Math.PI - theta);
        // rhat[1] = Math.sin(Math.PI - theta);
        // break;
        // case 7:
        // case 21:
        // rhat[0] = -1;
        // rhat[1] = 0;
        // break;
        // case 8:
        // case 20:
        // rhat[0] = Math.cos(theta);
        // rhat[1] = Math.sin(theta);
        // break;
        // case 9:
        // case 19:
        // break;
        // case 10:
        // case 18:
        // rhat[0] = 1;
        // rhat[1] = 0;
        // break;
        // case 11:
        // case 17:
        // break;
        // default:
        // //You're not seeing the right tag.
        // break;
        // }
    }

    @Override
    public void execute() {
        double speed = maxSpeed * -xPid.calculate(limelight.getX("limelight-left"));

        if (!limelight.hasTarget("limelight-left")) {
            speed = 0;
        }

        drive.drive(speed, 1, 0, 0, false); // If field relative boolean works correctly, above switch case is
                                            // unnecessary
        /*
         * If it doesn't,
         * Do:
         * drive.drive(speed, speed*rhat[0], speed*rhat[1], 0, true);
         */
    }

    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, 0, true);
    }

    public boolean isFinished() {
        return xPid.atSetpoint();
    }

}