// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class CanIdConstants {
    public static final int kGyroCanId = 15;
    public static final int kCoralCanId = 21;

    public static final int kAlgaeCanId = 22;
    public static final int kElevatorCanId = 23;
    public static final int kClimberCanId = 24;
    public static final int kPivotArmCanId = 25;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
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

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 3;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 14;
    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 13;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperaterControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  // all values with the value 8085 are placeholder as idk what im doing -Frank
  // THIS IS FROM LAST YEARS CODE MAY NEED UPDATING
  public static final class MotorDefaultsConstants {
    public static final int NeoCurrentLimit = 40;
    public static final int NeoVortexCurrentLimit = 60;
    public static final int Neo550CurrentLimit = 20;
    public static final MotorType NeoMotorType = MotorType.kBrushless;
    public static final MotorType Neo550MotorType = MotorType.kBrushless;
    public static final MotorType NeoVortexMotorType = MotorType.kBrushless;
  }

  public static final class ElevatorConstants {

  }

  public static final class PivotArmConstants {

  }

  public static final class CoralConstants {
    public static final int coralCurrentLimit = 20;
    public static final double kCoralSpeed = 1;

    // TEMPORARY VALUES
    public static double kCoralMinOutput = -0.25;
    public static double kCoralMaxOutput = 0.25;

    // TEMPORARY VALUES
    public static boolean kCoralInverted = true;
    // TEMPORARY VALUES
    public static int kCoralPositionConversionFactor = 1000;
    public static int kCoralVelocityConversionFactor = 1000;
    // TEMPORARY VALUES
    public static double kCoralP = 0.5;
    public static double kCoarlI = 0.0;
    public static double kCoralD = 0.0;
    public static double kCoralFF = 0.0;
  }

  public static final class AlgaeConstants {
    public static final int algaeCurrentLimit = 20;
    public static final double kAlgaeSpeed = 1;

    // TEMPORARY VALUES
    public static double kAlgaeMinOutput = -0.25;
    public static double kAlgaeMaxOutput = 0.25;

    // TEMPORARY VALUES
    public static boolean kAlgaeInverted = true;
    // TEMPORARY VALUES
    public static int kAlgaePositionConversionFactor = 1000;
    public static int kAlgaeVelocityConversionFactor = 1000;
    // TEMPORARY VALUES
    public static double kAlgaeP = 2.5;
    public static double kAlgaeI = 0.0;
    public static double kAlgaeD = 0.0;
    public static double kAlgaeFF = 0.0;
  }

  public static final class ClimberConstants {
    public static double kClimberSpeed = 0.8;
  }

  public static final class TuningModeConstants {
    public static boolean kAlgaeTuning = false;
    public static boolean kCoralTuning = true;
  }

}
