package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveAlignBarge extends Command {

    private DriveSubsystem driveSubsystem;
    private Pose2d targetPose;
    // private double kXYP = 0.5;
    // private double kXYI = 0;
    // private double kXYD = 0;
    // private double kRotP = 1.5;
    // private double kRotI = 0;
    // private double kRotD = 0;
    // x pid, y pid, and rotation pid
    private double kXP = 0.75; // 0.5
    private double kXI = 0;
    private double kXD = 0.1; // 0.1
    private double kYP = 0.75; // 0.5
    private double kYI = 0;
    private double kYD = 0.1; // 0.1
    private double kRotP = 1.7; // 3
    private double kRotI = 0;
    private double kRotD = 0.25; // 0.5

    private PIDController xPid = new PIDController(kXP, kXI, kXD, 0.02);
    private PIDController yPid = new PIDController(kXP, kXI, kXD, 0.02);
    private PIDController rotPid = new PIDController(kRotP, kRotI, kRotD, 0.02);

    public SwerveDriveAlignBarge(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Started Align Barge");
        Pose2d currentPose = this.driveSubsystem.getPose(); // get the current pose of the robot
        double targetX = 7.62; // Enter in the target X -- where we want the robot to end up - replace with
                               // correct number
        Rotation2d targetRot = Rotation2d.fromDegrees(-70); // This is the final rotation we want to be at

        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            targetX = 17.55 - targetX;
            targetRot = Rotation2d.fromDegrees(110);
        }

        // We want to go to specified X and current Y
        this.targetPose = new Pose2d(targetX, currentPose.getTranslation().getY(), targetRot);

        double[] targetPoseArray = { this.targetPose.getTranslation().getX(), this.targetPose.getTranslation().getY(),
                this.targetPose.getRotation().getRadians() };

        SmartDashboard.putNumberArray("Target Align Pose", targetPoseArray);
        this.rotPid.reset();
        this.xPid.reset();
        this.yPid.reset();
        this.rotPid.enableContinuousInput(-180, 180);

    }

    public double boundAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) {
            degrees -= 360;
        }
        return degrees;
    }

    @Override
    public void execute() {
        // run the Pid controllers using the targetpose we generated earlier

        // use the current pose of the robot as the input
        Pose2d currentPose = this.driveSubsystem.getPose();

        double currentRotation = this.boundAngle(currentPose.getRotation().getDegrees());
        double targetRotation = this.boundAngle(this.targetPose.getRotation().getDegrees());

        double vx = this.xPid.calculate(currentPose.getTranslation().getX(),
                targetPose.getTranslation().getX());
        double vy = this.yPid.calculate(currentPose.getTranslation().getY(),
                targetPose.getTranslation().getY());
        double vrot = this.rotPid.calculate(Math.toRadians(currentRotation), Math.toRadians(targetRotation));

        double speedVal = Math.sqrt(vx * vx + vy * vy);
        double forward = Math.pow(vx, 3);
        double right = Math.pow(vy, 3);
        double rotation = vrot;

        this.driveSubsystem.drive(
                speedVal,
                forward,
                right,
                rotation,
                true);
    }

    @Override
    public boolean isFinished() {
        // return if the Pid controller is at the target pose
        return this.xPid.atSetpoint() && this.yPid.atSetpoint() && this.rotPid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Align Barge Completed");

    }

}