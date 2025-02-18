package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX m_elevatorMotor = new TalonFX(Constants.CanIdConstants.kElevatorCanId, "rio"); // device id and
                                                                                                       // canbus
  TalonFXConfiguration config = new TalonFXConfiguration();
  private final CANcoder m_elevatorEncoder = new CANcoder(Constants.CanIdConstants.kElevatorCancoderCanID);
  private StatusSignal<Angle> elevatorPosition;
  private StatusSignal<AngularVelocity> elevatorVelocity;

  private MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0);

  private MotionMagicVelocityVoltage motionMagicVelocityControl = new MotionMagicVelocityVoltage(0);

  // Limit Switches
  DigitalInput topLimitSwitch = new DigitalInput(0);
  DigitalInput bottomLimitSwitch = new DigitalInput(2);
  DigitalInput zeroLimitSwitch = new DigitalInput(1);

  public boolean ElevatorLowerLimitHit() {
    return bottomLimitSwitch.get();
  }

  public boolean ElevatorRaiseLimitHit() {
    return topLimitSwitch.get();
  }

  public boolean ElevatorZeroLimitHit() {
    return zeroLimitSwitch.get();
  }

  public ElevatorSubsystem() {

    // Defining Elevator PIDs
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ElevatorConstants.kElevatorP;
    slot0Configs.kI = ElevatorConstants.kElevatorI;
    slot0Configs.kD = ElevatorConstants.kElevatorD;
    slot0Configs.kV = ElevatorConstants.kElevatorV;
    slot0Configs.kA = ElevatorConstants.kElevatorA;

    // Setting Motor Configs
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Setting Motion Magic Configs
    config.MotionMagic.MotionMagicCruiseVelocity = Constants.ElevatorConstants.kElevatorMMVelo;
    config.MotionMagic.MotionMagicAcceleration = Constants.ElevatorConstants.kElevatorMMAcc;
    config.MotionMagic.MotionMagicJerk = Constants.ElevatorConstants.kElevatorMMJerk;

    // get sensor readings
    elevatorPosition = m_elevatorMotor.getPosition();
    elevatorVelocity = m_elevatorEncoder.getVelocity();

    // Apply Configs
    m_elevatorMotor.getConfigurator().apply(config);
    m_elevatorMotor.getConfigurator().apply(slot0Configs);

  }

  // private double rotationsToMotorPosition(double rotations) {
  // return rotations * Constants.ElevatorConstants.kElevatorMotorGearRatio;
  // }

  // private double motorPositionToInches(double motorPosition) {
  // return motorPosition / Constants.ElevatorConstants.kElevatorMotorGearRatio;
  // }

  // Setting the height of the elevator
  // public void setPos(double rotations) {
  public void setPos(double rotations) {

    // Checks to see if the elevator height is below the minimum, and if it is, set
    // it to the minimum and
    // check to see if the elevator height is above the maximum, and if it is, set
    // it to the maximum

    // if we are at top limit set elevator to max, if we are at bottom set elevator
    // to min)
    if ((rotations < Constants.ElevatorConstants.kElevatorMin) /* || bottomLimitSwitch.get() */) {
      rotations = Constants.ElevatorConstants.kElevatorMin;
    } else if ((rotations > Constants.ElevatorConstants.kElevatorMax) /* || topLimitSwitch.get() */) {
      rotations = Constants.ElevatorConstants.kElevatorMax;
    }

    // set the feedforward value based on the elevator height
    double ff = 0;
    if (rotations > ElevatorConstants.kElevatorStage2Height) {
      ff = ElevatorConstants.kElevatorStage3FF;
    } else if (rotations > Constants.ElevatorConstants.kElevatorStage1Height) {
      ff = ElevatorConstants.kElevatorStage2FF;
    }

    // motionMagicControl.Position = rotationsToMotorPosition(rotations);
    motionMagicPositionControl.Position = rotations;
    motionMagicVelocityControl.FeedForward = ff;
    m_elevatorMotor.setControl(motionMagicPositionControl);
    // SmartDashboard.putNumber("height value", rotations);
    // SmartDashboard.putNumber("motor value", rotationsToMotorPosition(rotations));
  }

  // set the zero value of the motor encoder
  // public void zero(double rotations) {
  // m_elevatorMotor.setPosition(
  // rotations * Constants.ElevatorConstants.kElevatorMotorGearRatio);
  // }

  public void zero(double rotations) {
    m_elevatorMotor.setPosition(
        rotations);
  }

  // read the current elevator encoder position
  public double getCurrentMotorPosition() {
    elevatorPosition.refresh();
    return elevatorPosition.getValueAsDouble();
  }

  // read the current elevator encoder velocity
  public double getCurrentMotorVelocity() {
    elevatorVelocity.refresh();
    return elevatorVelocity.getValueAsDouble();
  }

  // // translate the current elevator encoder position into elevator height
  // public double getCurrentHeight() {
  // // return (motorPositionToInches(getCurrentMotorPosition()));
  // return getCurrentMotorPosition();
  // }

  public boolean targetInDangerZone(double target_position) {
    return target_position < Constants.ElevatorConstants.kElevatorSafeHeightMax;
  }

  public double minConflictHeight(Rotation2d target_angle) {
    var deg = Math.abs(target_angle.getRadians());
    var minHeight = 0.004 * deg + Constants.ElevatorConstants.kElevatorSafeHeightMin;
    if (minHeight < Constants.ElevatorConstants.kElevatorSafeHeightMin) {
      minHeight = Constants.ElevatorConstants.kElevatorSafeHeightMin;
    } else if (minHeight > Constants.ElevatorConstants.kElevatorSafeHeightMax) {
      minHeight = Constants.ElevatorConstants.kElevatorSafeHeightMax;
    }
    return minHeight;
  }

  public boolean targetInConflictZone(double target_position, Rotation2d target_angle) {
    return target_position < minConflictHeight(target_angle);
  }

  public boolean inDangerZone() {
    return targetInDangerZone(getCurrentMotorPosition());
  }

  public boolean atTarget(double tolerance_rotations) {
    return Math.abs(getCurrentMotorPosition() - motionMagicPositionControl.Position) < tolerance_rotations;
  }

  public void periodic() {
    // display encoder readings on dashboard
    SmartDashboard.putNumber("current Elevator Motor Position", getCurrentMotorPosition());
    // SmartDashboard.putNumber("current Height", getCurrentHeight());
    SmartDashboard.putBoolean("top limit switch hit", topLimitSwitch.get());
    SmartDashboard.putBoolean("bottom limit switch hit", bottomLimitSwitch.get());
    SmartDashboard.putBoolean("zero limit switch hit", zeroLimitSwitch.get());

  }

  // turn off elevator (stops motor all together)
  public void stop() {
    m_elevatorMotor.set(0);
  }

  // open loop move elevator up & down
  public void moveDown() {
    m_elevatorMotor.set(-ElevatorConstants.kElevatorSpeed);
  }

  public void moveUp() {
    m_elevatorMotor.set(ElevatorConstants.kElevatorSpeed);
  }

  public void holdHeight() {
    // double targetPosition = getCurrentHeight();
    double targetPosition = getCurrentMotorPosition();
    setPos(targetPosition);
  }
}
