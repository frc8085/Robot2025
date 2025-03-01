// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TuningModeConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class LimelightSubsystem extends SubsystemBase {
    private boolean TUNING_MODE = TuningModeConstants.kLimelightTuning;
    // private boolean PRACTICE_MODE = TuningModeConstants.kPracticeMode;

    /** Creates a new LimelightSubsystem. */
    DriveSubsystem m_drive;

    // public static HttpCamera m_limelight;

    private boolean m_visionMode;

    public LimelightSubsystem(
            DriveSubsystem drive) {
        m_drive = drive;

        // m_limelight = new HttpCamera("LL", "http://limelight:5809/stream.mjpg");
        // m_limelight.setResolution(320, 240);
        // m_limelight.setFPS(90);

        // CameraServer.addCamera(m_limelight);
    }

    // public static double getAprilTagID() {
    // // The computer fought with us :( thats why it looks so weird
    // double[] id =
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid")
    // .getDoubleArray(new double[6]);
    // return id[0];
    // };

    // public static double[] getRobotLocation() {
    // // gets robot position realtive to AprilTag
    // double[] location = NetworkTableInstance.getDefault().getTable("limelight")
    // .getEntry("botpose_targetspace")
    // .getDoubleArray(new double[6]);
    // return location;
    // }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // read values periodically
        // SmartDashboard.putData(SendableCameraWrapper.wrap(m_limelight));
        // SmartDashboard.putData(SendableCameraWrapper.wrap(m_limelightRight));
        // limelight-left, limelight-right
        if (TUNING_MODE || true) {
            // SmartDashboard.putNumber("April tag ID", getAprilTagID("limelight-left"));
            SmartDashboard.putBoolean("Target", hasTarget("limelight-left"));
            SmartDashboard.putNumber("LL ID", getID("limelight-left"));

            SmartDashboard.putNumber("LL X", getX("limelight-left"));
            SmartDashboard.putNumber("LL Y", getY("limelight-left"));

        }

    }

    public double getX(String limelightName) {
        return LimelightHelpers.getTX(limelightName);
    }

    // Account for the Robot's current heading when returning the X value
    public double getXfromRobotPerspective() {
        // Heading from -180 to 180
        // double robotHeading = m_drive.getHeadingWrappedDegrees();
        // double degreesToTarget = -getX();
        // double degreesFromRobotPerspective = robotHeading + degreesToTarget;
        // return degreesFromRobotPerspective;
        return 0;
    }

    public double getY(String limelightName) {
        return LimelightHelpers.getTY(limelightName);
    }

    public double getDistance(String limelightName) {
        return LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getZ();
    }

    public double getRotation(String limelightName) {
        double[] botPose = LimelightHelpers.getBotPose(limelightName);
        return botPose[5]; // Yaw rotation (degrees)

    }

    // Account for the robot's current shooter angle when returning the Y value
    public double getYfromRobotPerspective() {
        // double currentShooterPivotPosition = m_arm.getShooterPivotPosition();
        // double idealY = -6;
        // double degreesToTarget = -(getY() - idealY);
        // double degreesToAdjustShooterPivot = currentShooterPivotPosition +
        // degreesToTarget;
        // return degreesToAdjustShooterPivot;
        return 0;
    }

    public double getArea(String limelightName) {
        return LimelightHelpers.getTA(limelightName);
    }

    public double getShooterPivotSetpointFromArea() {
        // Empricially Derived
        // pivotSetpoint = -44.3* ta + 72.6
        // adjusted based on new TA readings and working setpoints
        /**
         * TA - Setpoint
         * 0.21 59 - true measurement 61, but adjusted down because we were missing low
         * at far distances
         * 0.26 57
         * 0.32 55
         * 0.5 50
         **/

        // double ta = getArea();
        // return -30.4 * ta + 65;
        return 0;
    }

    public boolean hasTarget(String limelightName) {
        return LimelightHelpers.getTV(limelightName);
    }

    public int getID(String limelightName) {
        return (int) LimelightHelpers.getFiducialID(limelightName);
    }

    // public double getLatPip() {
    // return table.getEntry("tl").getDouble(0.0) / 1000.0;
    // }

    // public double getLatCap() {
    // return table.getEntry("cl").getDouble(0.0) / 1000.0;
    // }

    // public double getLastEntryTimeStamp() {
    // return Timer.getFPGATimestamp() - getLatCap() - getLatPip();
    // }

    /**
     * @param piplineNumber driver = 0, aprilTags = 1, retroreflective = 2
     */
    // public void setPipeline(int pipelineNumber) {
    // Number numObj = (Number) pipelineNumber;
    // table.getEntry("pipeline").setNumber(numObj);
    // }

    public boolean inVisionMode() {
        return m_visionMode;
    }

    public void setVisionModeOn() {
        m_visionMode = true;
    }

    public void setVisionModeOff() {
        m_visionMode = false;
    }
}
