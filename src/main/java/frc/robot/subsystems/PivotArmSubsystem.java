package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.PivotArmConstants;

public class PivotArmSubsystem extends SubsystemBase {
  private final TalonFX m_elevatorMotor = new TalonFX(CanIdConstants.kPivotArmCanId, "rio"); // change deviceID and canbus
  TalonFXConfiguration config = new TalonFXConfiguration();

  private double kSpeed = PivotArmConstants.kPivotArmSpeed;

  public PivotArmSubsystem() {
    // Add configurations to Configs.java
    // https://pbs.twimg.com/media/F1Zwg4HacAEepQn.jpg:large
    m_elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    config.Slot0.kP = PivotArmConstants.kPivotArmP;
    config.Slot0.kI = PivotArmConstants.kPivotArmI;
    config.Slot0.kD = PivotArmConstants.kPivotArmD;
    
    m_elevatorMotor.getConfigurator().apply(config);

  }

  public void moveUp() {
    // create a position closed-loop request, voltage output, slot 0 configs
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    // set position to 10 rotations
    m_elevatorMotor.setControl(m_request.withPosition(10));
  }

  public void stop() {
    m_elevatorMotor.set(0);
  }

  public void moveDown() {
    m_elevatorMotor.set(-kSpeed);
  }

  public void start() {
    m_elevatorMotor.set(kSpeed);
  }
}
