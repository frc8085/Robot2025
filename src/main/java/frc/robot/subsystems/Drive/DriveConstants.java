package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {

    // Drive CanIDs
    public static final int kGyroCanId = 15;

    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 13;
    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 14;

    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 1; // Rev stated max speed

    // what is the multiplier for the speed decrease
    public static final double kMinSpeedMetersPerSecondMaxElevatorHeightMul = 0.05;

    public static final double kMinSpeedMetersPerSecondMaxElevatorHeight = 0.2;
    public static final double kSlowDrive = 0.1;

    // if you want to slow down the rotation speed, change the adjustment factor
    public static final double kAngularSpeedAdjustment = .5;
    public static final double kMaxAngularSpeed = 2 * Math.PI * kAngularSpeedAdjustment; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final boolean kGyroReversed = false;

    // Copied from 6616
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    // TODO: Update this value
    public static final double GYRO_OFFSET = 0;

    // Enum for auto-orienting to field directions
    public enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT
    }

    public static final class FakeConstants {
        public static boolean fieldRelative = true;
    }

}
