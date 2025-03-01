
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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
    public static final int kPivotGyroCanId = 16;

    public static final int kCoralCanId = 21;
    public static final int kAlgaeCanId = 22;
    public static final int kClimberCanId = 24;

    public static final int kElevatorCanId = 23;
    public static final int kPivotArmCanId = 25;

    public static final int kElevatorCancoderCanID = 33;
    public static final int kPivotArmCancoderCanID = 35;
  }

  public static final class DriveConstants {
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8 / 2; // 4.8

    // what is the multiplier for the speed decrease
    public static final double kMinSpeedMetersPerSecondMaxElevatorHeightMul = 0.025;

    public static final double kMinSpeedMetersPerSecondMaxElevatorHeight = 0.2;

    // if you want to slow down the rotation speed, change the adjustment factor
    public static final double kAngularSpeedAdjustment = 1;
    public static final double kMaxAngularSpeed = 2 * Math.PI * kAngularSpeedAdjustment; // radians per second

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
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 13;
    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 14;

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
    public static final double kElevatorMotorGearRatio = 5;
    public static double kElevatorSpeed = .25;
    public static double kElevatorP = 4;
    public static double kElevatorI = 0;
    public static double kElevatorD = 0;
    public static double kElevatorV = .12;
    public static double kElevatorA = .01;
    public static double kElevatorStage2FF = 0.19;
    public static double kElevatorStage3FF = 0.38;

    public static double kElevatorMMVelo = 120;
    public static double kElevatorMMAcc = 140;
    public static double kElevatorMMJerk = 1600;

    // Elevator Heights for different states
    public static double kElevatorHomeHeight = 30;
    public static double kElevatorCoralPickupHeight = 20;
    public static double kElevatorCoralPickupAlternateHeight = 15;
    public static double kElevatorCoralDropOff1Height = 30;
    public static double kElevatorCoralDropOff2Height = 60;
    public static double kElevatorCoralDropOff3Height = 88;
    public static double kElevatorCoralDropOff4Height = 130;
    public static double kElevatorAlgaePickUpFloorHeight = 11;
    public static double kElevatorReef2Height = 40;
    public static double kElevatorReef3Height = 70;
    public static double kElevatorAlgaePickUpFloorFlipHeight = 6;
    public static double kElevatorReef2IntakeHeight = 30;
    public static double kElevatorReef3IntakeHeight = 45;
    public static double kElevatorNetHeight = 120;

    // Determine what actual height values these are and/or what encoder readings
    // Stage Height refers to top of stage value
    public static final double kElevatorMin = 0; // adjusting for climber
    public static final double kElevatorStage1Height = 41; // zero position value
    public static final double kElevatorStage2Height = 85;
    public static final double kElevatorMax = 130; // 140 hard max

    /// The minimum height of the elevator that the pivot arm can swing through
    public static final double kElevatorSafeHeightMax = 55;
    public static final double kElevatorSafeHeightMin = 15;

    // The maximum height that the robot can safely travel at
    public static final double kElevatorSafeTravelHeight = 50;
    public static final double kElevatorSafeMidSpeedTravelHeight = 50;
    // this variable determines the minimum height at which the drivetrain speed
    // will be
    // its
    // slowest
    public static final double kElevatorMinTravelHeight = 80;

    // the Elevator tolerance
    public static final double kElevatorTolerance = 5;
  }

  public static final class PivotArmConstants {

    public static final double kPivotMotorGearRatio = 27;
    public static double kPivotArmSpeed = .10;

    public static final double kPivotArmP = 1.4; // 0.7
    public static final double kPivotArmI = 0; // 0.0
    public static final double kPivotArmD = 0.1; // 0.1
    public static final double kPivotArmS = 0.07;
    public static final double kPivotArmV = 0.2; // 0.2
    public static final double kPivotArmA = 0.015; // 0.015
    public static final double kPivotArmFF = -0.13;

    public static double kPivotArmMMVelo = 25;
    public static double kPivotArmMMAcc = 20;
    public static double kPivotArmMMJerk = 1600;

    public static final Rotation2d kPivotArmMin = Rotation2d.fromDegrees(-110);
    public static final Rotation2d kPivotArmMax = Rotation2d.fromDegrees(120);

    public static final Rotation2d kPivotArmMaxManual = Rotation2d.fromDegrees(460);
    public static final Rotation2d kPivotArmMinManual = Rotation2d.fromDegrees(-270);

    /// The min/max angle of the pivot that will be rotating through the path of the
    /// elevator
    public static final Rotation2d kPivotArmSwingThroughMax = Rotation2d.fromDegrees(35);
    public static final Rotation2d kPivotArmSwingThroughMin = Rotation2d.fromDegrees(-35);

    // the Tolerance for pivot command motion
    public static final Rotation2d kPivotTolerance = Rotation2d.fromDegrees(5);
    public static final double kPivotToleranceRotations = kPivotTolerance.getRotations();

    public static final double kAlgaeNetLeftPivot = 60;
    public static final double kAlgaeNetRightPivot = 0;

  }

  public static final class Windmill {

    public static enum WindmillState {

      Home(ElevatorConstants.kElevatorHomeHeight, Rotation2d.fromDegrees(45)),
      Climb(ElevatorConstants.kElevatorStage1Height, Rotation2d.fromDegrees(90)),
      CoralPickup(ElevatorConstants.kElevatorCoralPickupHeight, Rotation2d.fromDegrees(120)),
      CoralPickupAlternate(ElevatorConstants.kElevatorCoralPickupAlternateHeight, Rotation2d.fromDegrees(120)),

      // coral dropoff happens on both sides
      CoralDropOff1(ElevatorConstants.kElevatorCoralDropOff1Height, Rotation2d.fromDegrees(-50), true),
      CoralDropOff2(ElevatorConstants.kElevatorCoralDropOff2Height, Rotation2d.fromDegrees(-50), true),
      CoralDropOff3(ElevatorConstants.kElevatorCoralDropOff3Height, Rotation2d.fromDegrees(-50), true),
      CoralDropOff4(ElevatorConstants.kElevatorCoralDropOff4Height, Rotation2d.fromDegrees(-55), true),

      AlgaePickUpFloor(ElevatorConstants.kElevatorAlgaePickUpFloorHeight, Rotation2d.fromDegrees(116)),
      AlgaePickUpReef2(ElevatorConstants.kElevatorReef2Height, Rotation2d.fromDegrees(102)),
      AlgaePickUpReef3(ElevatorConstants.kElevatorReef3Height, Rotation2d.fromDegrees(102)),
      AlgaePickUpFloorFlip(ElevatorConstants.kElevatorAlgaePickUpFloorFlipHeight, Rotation2d.fromDegrees(90)),
      AlgaePickUpReef2Flip(ElevatorConstants.kElevatorReef2IntakeHeight, Rotation2d.fromDegrees(-35)),
      AlgaePickUpReef3Flip(ElevatorConstants.kElevatorReef3IntakeHeight, Rotation2d.fromDegrees(-25)),
      AlgaeNetLeft(ElevatorConstants.kElevatorNetHeight, Rotation2d.fromDegrees(PivotArmConstants.kAlgaeNetLeftPivot)),
      AlgaeNetRight(ElevatorConstants.kElevatorNetHeight,
          Rotation2d.fromDegrees(PivotArmConstants.kAlgaeNetRightPivot));

      private double kElevatorHeight;
      private Rotation2d kPivotArmAngle;
      private boolean canMirror;

      private WindmillState(double kElevatorHeight, Rotation2d kPivotArmAngle) {
        this.kElevatorHeight = kElevatorHeight;
        this.kPivotArmAngle = kPivotArmAngle;
        this.canMirror = false;
      }

      private WindmillState(double kElevatorHeight, Rotation2d kPivotArmAngle, boolean canMirror) {
        this.kElevatorHeight = kElevatorHeight;
        this.kPivotArmAngle = kPivotArmAngle;
        this.canMirror = canMirror;
      }

      public double getElevatorHeight() {
        return kElevatorHeight;
      }

      public Rotation2d getPivotArmAngle() {
        return kPivotArmAngle;
      }

      public boolean canMirror() {
        return canMirror;
      }

    }
  }

  public static final class CoralConstants {
    public static final int coralCurrentLimit = 40;
    public static final double kCoralSpeed = 1;
    public static final double kCoralSlowSpeed = .5;

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
    public static double kCoralI = 0.0;
    public static double kCoralD = 0.0;
    public static double kCoralFF = 0.0;

    // Beam Break Sensor
    public static int kIRPort = 3;
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

    // Beam Break Sensor
    public static int kIRPort = 4;

  }

  public static final class ClimberConstants {
    public static double kWinchSpeed = 0.4;
    public static double kWinchP = 0;
    public static double kWinchI = 0;
    public static double kWinchD = 0;
    public static double kWinchFF = 0;
    public static double kWinchMinOutput = 0;
    public static double kWinchMaxOutput = 1;
  }

  public static final class TuningModeConstants {
    public static boolean kLimelightTuning = false;
    public static boolean kAlgaeTuning = false;
    public static boolean kCoralTuning = false;
    public static boolean kElevatorTuning = true;
    public static boolean kPivotTuning = true;
  }

}
