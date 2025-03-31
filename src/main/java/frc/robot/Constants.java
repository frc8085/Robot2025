
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Pivot.*;
import frc.robot.subsystems.Elevator.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;

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

  public static final class TuningModeConstants {
    public static boolean kLimelightTuning = false;
    public static boolean kAlgaeTuning = false;
    public static boolean kCoralTuning = false;
    public static boolean kElevatorTuning = true;
    public static boolean kPivotTuning = true;
    public static boolean kClimberTuning = true;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
    public static final double kTurnDeadband = 0.015;
    public static final double kDpadSpeedRegulator = 0.25;

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

    public static final double ROBOT_MASS_KG = 62.14;
    public static final double ROBOT_MOI = 8.66;
    public static final double WHEEL_COF = 0.0484;

    // TODO: Tune these values
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.01, 0, 0);

    public static final PathFollowingController PP_CONTROLLER = new PPHolonomicDriveController(
        TRANSLATION_PID, // new PIDConstants(5, 0.0, 0.0), // Translation PID constants
        ANGLE_PID); // new PIDConstants(5, 0.0, 0.0)); // Rotation PID constants

    // TODO: Update these values
    public static final double LIMELIGHT_HEIGHT_METERS = 0.254;
    public static final double LIMELIGHT_MOUNTING_ANGLE_DEGREES = 30.0;
    public static final double LIMELIGHT_MOUNTING_ANGLE_RADIANS = Math
        .toRadians(AutoConstants.LIMELIGHT_MOUNTING_ANGLE_DEGREES);

    public static final double REEF_APRILTAG_HEIGHT = 0.324;

    // X offset, Y offset, Rotation offset
    public static final double leftYOffset = 0;
    public static final double rightYOffset = -0.25;
    // for offsets, x positive is backwards from the apriltag, y positive is to the
    // right of the apriltag
    // so for example, and x of 1 and a y of 1 would be 1 meter back and 1 meter to
    // the right of the apriltag
    public static final Pose2d leftReefAlignPose = new Pose2d(0.4, -0.30 + leftYOffset, Rotation2d.kZero);
    public static final Pose2d rightReefAlignPose = new Pose2d(0.4, 0.30 + rightYOffset, Rotation2d.kZero);

    public static final Rotation2d rotationBlue = Rotation2d.fromDegrees(90);
    public static final Rotation2d rotationYellow = Rotation2d.fromDegrees(-90);

  }

  public static final class MotorDefaultsConstants {
    public static final int NeoCurrentLimit = 40;
    public static final int NeoVortexCurrentLimit = 60;
    public static final int Neo550CurrentLimit = 20;
    public static final MotorType NeoMotorType = MotorType.kBrushless;
    public static final MotorType Neo550MotorType = MotorType.kBrushless;
    public static final MotorType NeoVortexMotorType = MotorType.kBrushless;
    public static final double NeoMotorFreeSpeedRpm = 5676;

  }

  public static final class Windmill {

    public static enum WindmillState {

      Home(ElevatorConstants.kElevatorHomeHeight, Rotation2d.fromDegrees(PivotArmConstants.kPivotHome)),
      Climb(ElevatorConstants.kElevatorStage1Height, Rotation2d.fromDegrees(PivotArmConstants.kPivotClimb)),
      LimelightYellowTarget(ElevatorConstants.kElevatorHomeHeight, Rotation2d.fromDegrees(74)),
      AlgaeHoldHeight(ElevatorConstants.kElevatorAlgaeHoldHeight, Rotation2d.fromDegrees(PivotArmConstants.kPivotHome)),
      CoralPickup(ElevatorConstants.kElevatorCoralPickupHeight,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotCoralPickup)),
      CoralPickupAlternate(ElevatorConstants.kElevatorCoralPickupAlternateHeight,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotCoralPickup)),
      CoralPickupHigher(ElevatorConstants.kElevatorCoralPickupHigher,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotCoralPickup)),
      CoralPickupAltHigher(ElevatorConstants.kElevatorCoralPickupAltHigher,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotCoralPickup)),
      AutoTravel(ElevatorConstants.kElevatorAutoTravelHeight, Rotation2d.fromDegrees(PivotArmConstants.kPivotHome)),
      // coral dropoff happens on both sides
      CoralDropOff1(ElevatorConstants.kElevatorCoralDropOff1Height,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotCoralDropOff1), true),
      CoralDropOff2(ElevatorConstants.kElevatorCoralDropOff2Height,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotCoralDropOff), true),
      CoralDropOff3(ElevatorConstants.kElevatorCoralDropOff3Height,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotCoralDropOff), true),
      CoralDropOff4(ElevatorConstants.kElevatorCoralDropOff4Height,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotCoralDropOff4), true),

      AlgaeProcessor(ElevatorConstants.kElevatorAlgaePickUpFloorHeight,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotAlgaeProcessor)),

      AlgaePickUpFloor(ElevatorConstants.kElevatorAlgaePickUpFloorHeight,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotAlgaePickUpFloor)),
      AlgaePickUpReef2(ElevatorConstants.kElevatorReef2Height, Rotation2d.fromDegrees(PivotArmConstants.kPivotReef)),
      AlgaePickUpReef3(ElevatorConstants.kElevatorReef3Height, Rotation2d.fromDegrees(PivotArmConstants.kPivotReef)),
      AlgaePickUpFloorFlip(ElevatorConstants.kElevatorAlgaePickUpFloorFlipHeight,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotAlgaePickUpFloorFlip)),
      AlgaePickUpReef2Flip(ElevatorConstants.kElevatorReef2FlipHeight,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotReef2Flip)),
      AlgaePickUpReef3Flip(ElevatorConstants.kElevatorReef3FlipHeight,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotReef3Flip)),
      AlgaeNetBlue(ElevatorConstants.kElevatorNetHeight, Rotation2d.fromDegrees(PivotArmConstants.kPivotAlgaeNetBlue)),
      AlgaeNetYellow(ElevatorConstants.kElevatorNetHeight,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotAlgaeNetYellow)),
      AlgaeNetRaise(ElevatorConstants.kElevatorNetHeight,
          Rotation2d.fromDegrees(PivotArmConstants.kPivotAlgaeNetRaise));

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

  public static final class CommandScoreConstants {
    public static double kMoveSpeed = 0.5;
  }

}
