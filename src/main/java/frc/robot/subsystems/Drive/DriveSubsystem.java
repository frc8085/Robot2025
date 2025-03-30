// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.States.DriveState;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.Limelight.LimelightHelpers.PoseEstimate;
import choreo.trajectory.SwerveSample;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // Drive PIDS
  private double kXP = 1.5; // 0.5
  private double kXI = 0;
  private double kXD = 0.1; // 0.1
  private double kYP = 1.5; // 0.5
  private double kYI = 0;
  private double kYD = 0.1; // 0.1
  private double kRotP = 3; // 3
  private double kRotI = 0;
  private double kRotD = 0.25; // 0.5

  private PIDController xController = new PIDController(kXP, kXI, kXD, 0.02);
  private PIDController yController = new PIDController(kYP, kYI, kYD, 0.02);
  private PIDController headingController = new PIDController(kRotP, kRotI, kRotD, 0.02);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGyroCanId);

  // This section was copied from 2024 code, I think for auto
  public Field2d field = new Field2d();

  // Copied from 6616 - PID Controller for orientation to supplied angle
  private final PIDController orientationController;

  // Odometry class for tracking robot pose
  // SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
  // DriveConstants.kDriveKinematics,
  // getYaw(),
  // getModulePositions(),
  // new Pose2d(0, 0, new Rotation2d(0)));
  SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      this.getGyroOrientation(),
      getModulePositions(), new Pose2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    // configurePathPlanner();
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) ->

          driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                                      // optionally outputs individual module feedforwards
          AutoConstants.PP_CONTROLLER,
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // from 6616
    orientationController = new PIDController(AutoConstants.ANGLE_PID.kP, AutoConstants.ANGLE_PID.kI,
        AutoConstants.ANGLE_PID.kD);
    orientationController.enableContinuousInput(-180, 180);

    SmartDashboard.putBoolean("useMegatag2", true);

  }

  public void inputCameraPoses(String[] limelightNames) {
    for (String limelightName : limelightNames) {

      LimelightHelpers.SetRobotOrientation(limelightName, this.getGyroOrientation().getDegrees(),
          m_gyro.getAngularVelocityZWorld().getValueAsDouble(), 0, 0, 0, 0);

      if (LimelightHelpers.getTV(limelightName)) {
        // Pose2d pose = limelight.getBotPoseBlue(limelightName);
        PoseEstimate poseEstimate;

        boolean megatag2 = SmartDashboard.getBoolean("useMegatag2", true);

        if (megatag2) { // REMEMBER TO CHANGE THIS BACK
          poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        } else {
          // pose = LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
          poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        }
        Pose2d pose = poseEstimate.pose;
        double[] limelightArrayData = { pose.getX(), pose.getY(),
            pose.getRotation().getRadians() };
        SmartDashboard.putNumberArray(limelightName, limelightArrayData);
        // System.out.println("Limelight pose: " + pose);
        // double visionTime = Timer.getFPGATimestamp() -
        // limelight.getVisionTime(limelightName);
        double visionTime = Timer.getFPGATimestamp() - poseEstimate.latency / 1000;
        // int tagCount = limelight.getTagCount(limelightName);
        int tagCount = poseEstimate.tagCount;
        // int primaryId = limelight.getPrimaryId(limelightName);
        int primaryId = (int) LimelightHelpers.getFiducialID(limelightName);
        double distanceToTarget = poseEstimate.avgTagDist;
        this.addVisionPose(pose, visionTime, tagCount, primaryId, distanceToTarget, poseEstimate.isMegaTag2);
      }
    }
  }

  public void addVisionPose(Pose2d pose, double visionTime, int tagCount, int primaryId, double distanceToTarget,
      boolean isMegaTag2) {

    double stdDev = 0.4;
    double stdDevRot = 10.0;

    if (tagCount == 0) {
      return;
    }

    boolean primaryReef = false;
    if (6 <= primaryId && primaryId <= 11) {
      primaryReef = true;
    } else if (17 <= primaryId && primaryId <= 22) {
      primaryReef = true;
    }

    if (!isMegaTag2) {
      stdDev = 2;
      pose = new Pose2d(pose.getTranslation(), this.getRotation());

      if (tagCount == 1) {
        if (distanceToTarget > 2.5) { // meters
          return;
        }
        if (primaryReef && (distanceToTarget <= 1.5)) {
          stdDev = 0.25;
          if (distanceToTarget <= 0.75) {
            stdDev = 0.1;
            if (DriverStation.isTeleop()) {
              this.m_odometry.addVisionMeasurement(pose, visionTime,
                  VecBuilder.fill(stdDev, stdDev, stdDev));
              return;
            }
          }
        }
      }

      if (tagCount >= 2) {
        stdDev = 0.7;
        if (primaryReef && (distanceToTarget <= 0.5)) {
          stdDev = 0.5;
          if (distanceToTarget <= 0.25) {
            stdDev = 0.25;
          }
        }
      }

    } else {
      if ((tagCount > 1) || (distanceToTarget < 2) || (primaryReef && (distanceToTarget < 1))) {
        stdDev = 0.25;

      }
      stdDevRot = 5;

    }

    this.m_odometry.addVisionMeasurement(
        // new Pose2d(pose.getTranslation(),
        // Rotation2d.fromDegrees(getGyroOrientation())),
        pose,
        visionTime,
        VecBuilder.fill(stdDev, stdDev, stdDevRot)
    // VecBuilder.fill(0.1, 0.1, 0.1)
    );
    return;
  }

  @Override
  public void periodic() {

    // Update the odometry in the periodic block
    m_odometry.updateWithTime(Timer.getFPGATimestamp(),
        this.getGyroOrientation(),
        this.getModulePositions());

    this.inputCameraPoses(new String[] { "limelight-blue", "limelight-yellow" });

    SmartDashboard.putNumber("robot heading", getHeading());
    SmartDashboard.putNumber("robot wrapped heading", getHeadingWrappedDegrees());
    SmartDashboard.putBoolean("fieldRelative", DriveConstants.FakeConstants.fieldRelative);
    double[] poseData = { this.getPose().getX(), this.getPose().getY(), this.getPose().getRotation().getRadians() };
    SmartDashboard.putNumberArray("Robot Pose", poseData);
  }

  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d pose = this.getPose();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds = new ChassisSpeeds(
        (sample.vx + xController.calculate(pose.getX(), sample.x)),
        // sample.vx,
        (sample.vy + yController.calculate(pose.getY(), sample.y)),
        // sample.vy,
        sample.omega + headingController.calculate(pose.getRotation().getRadians(),
            sample.heading)
    // sample.omega
    );

    // Apply the generated speeds
    this.driveAutoFieldRelative(speeds);
  }

  /**
   * Drive in a robot relative direction.
   * Accepts ChassisSpeeds object for path planner integration.
   * 
   * @param robotRelativeSpeeds
   */
  private void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates((targetSpeeds));
    setModuleStates(targetStates);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  public void stop() {
    drive(0, 0, 0, 0, false);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose (x / y coordinates and rotation).
   */
  public Pose2d getPose() {
    // return m_odometry.getPoseMeters(); // this was using PoseEstimator
    return m_odometry.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return this.getPose().getRotation();
  }

  /**
   * Returns the currently-estimated rotation of the robot bound between -180 and
   * 180
   *
   * @return The rotation
   */
  public double getHeading() {
    // return m_odometry.getPoseMeters(); // this was using PoseEstimator
    return MathUtil.inputModulus(this.getRotation().getDegrees(), -180, 180);
  }

  public void resetOdometry(Pose2d pose) {
    // this.m_odometry.resetPosition(this.getGyroOrientation(),
    // this.getModulePositions(), pose);
    this.m_odometry.resetPosition(Rotation2d.kZero, this.getModulePositions(), new Pose2d());

    Rotation2d gyroOffset = pose.getRotation().minus(this.getRotation());
    // this.m_gyro.setYaw(gyroOffset.getDegrees());

    this.m_gyro.setYaw(pose.getRotation().getDegrees());
    this.m_odometry.resetTranslation(pose.getTranslation());
  }

  // public void resetOdometryAuto(Pose2d pose) {
  // this.m_odometry.resetPosition(this.getRotation(), this.getModulePositions(),
  // pose);
  // }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param speed         How much of the right trigger is pressed.
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double speed, double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    if (xSpeed == 0 && ySpeed == 0) {
      speed = 0;
    }

    double dElevatorSpeed = DriveConstants.kMaxSpeedMetersPerSecond
        - DriveConstants.kMinSpeedMetersPerSecondMaxElevatorHeight;

    // Convert the commanded speeds into the correct units(angle) for the drivetrain
    double joystickAngle = Math.atan2(ySpeed, xSpeed);

    // allows trigger to apply speed and converts an angle into x and y speeds.
    double vy = speed * Math.sin(joystickAngle);
    double vx = speed * Math.cos(joystickAngle);

    double maxDrivableSpeed = DriveState.elevatorMultiplier * dElevatorSpeed
        + DriveConstants.kMinSpeedMetersPerSecondMaxElevatorHeight;

    SmartDashboard.putNumber("Max drivable speed",
        maxDrivableSpeed);

    double xSpeedDelivered = vx * maxDrivableSpeed;
    double ySpeedDelivered = vy * maxDrivableSpeed;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                // Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()))
                this.getGyroOrientation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    this.setModuleStates(swerveModuleStates);
  }

  private void driveAutoFieldRelative(ChassisSpeeds speeds) {
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getRotation());
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(fieldSpeeds);
    setModuleStates(moduleStates);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void lock() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(addForAlliance());
  }

  public void resetGyroBasedOnField() {
    // grabs the field
  }

  // Inverts the joystick direction if on red alliance
  public double invertForAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return -1;
    }
    return 1;
  }

  // Adds 180 degrees if on red alliance
  private double addForAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return 180;
    }
    return 0;
  }

  // /**
  // * Returns the heading of the robot.
  // *
  // * @return the robot's heading in degrees, from -180 to 180
  // */
  // public double getHeading() {
  // return SwerveUtils.normalizeAngle(getGyroOrientation());
  // }

  // /**
  // * Returns the turn rate of the robot.
  // *
  // * @return The turn rate of the robot, in degrees per second
  // */
  // public double getTurnRate() {
  // return m_gyro.getAngularVelocityXWorld().getValueAsDouble() *
  // (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }

  public double getHeadingWrappedDegrees() {
    return MathUtil.inputModulus(getHeading(), -180, 180);
  }

  private Rotation2d getGyroOrientation() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble() + DriveConstants.GYRO_OFFSET);
  }

}
