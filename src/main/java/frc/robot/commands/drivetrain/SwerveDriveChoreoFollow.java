package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.DriveSubsystem;
import choreo.trajectory.Trajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;

public class SwerveDriveChoreoFollow extends Command {

    private DriveSubsystem driveSubsystem;
    private Optional<Trajectory<SwerveSample>> trajectory;
    private Timer timer = new Timer();
    boolean isRedAlliance = false;
    boolean isFinished = false;
    boolean moveToStartPose = false;

    public SwerveDriveChoreoFollow(DriveSubsystem driveSubsystem, Optional<Trajectory<SwerveSample>> trajectory,
            boolean moveToStartPose) {
        this.driveSubsystem = driveSubsystem;
        this.trajectory = trajectory;
        this.moveToStartPose = moveToStartPose;
        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize() {
        this.isFinished = false;
        this.isRedAlliance = false;
        System.out.println("Auto Path Started");
        if (this.trajectory.isEmpty()) {
            System.out.println("Trajectory is empty");
            this.isFinished = true;
            return;
        }
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                this.isRedAlliance = true;
            }
        }

        Optional<Pose2d> initialPose = this.trajectory.get().getInitialPose(this.isRedAlliance);
        if (initialPose.isPresent() && this.moveToStartPose) {
            this.driveSubsystem.updatePose(initialPose.get());
        }

        double[] poses = new double[this.trajectory.get().getPoses().length * 3];
        Trajectory<SwerveSample> traj = this.trajectory.get();
        if (this.isRedAlliance) {
            traj = traj.flipped();
        }
        for (int i = 0; i < this.trajectory.get().getPoses().length; i++) {
            poses[i * 3] = traj.getPoses()[i].getTranslation().getX();
            poses[i * 3 + 1] = traj.getPoses()[i].getTranslation().getY();
            poses[i * 3 + 2] = traj.getPoses()[i].getRotation().getDegrees();
        }

        SmartDashboard.putNumberArray("Choreo Traj", poses);

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {

        Optional<SwerveSample> currentSample = this.trajectory.get().sampleAt(timer.get(), this.isRedAlliance);

        if (currentSample.isPresent()) {

            Pose2d currentPose = currentSample.get().getPose();

            SmartDashboard.putNumberArray("Choreo Path", new double[] { currentPose.getTranslation().getX(),
                    currentPose.getTranslation().getY(), currentPose.getRotation().getDegrees() });

            this.driveSubsystem.followTrajectory(currentSample.get());
        }

        if (timer.hasElapsed(trajectory.get().getTotalTime())) {
            this.isFinished = true;
        }

    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}