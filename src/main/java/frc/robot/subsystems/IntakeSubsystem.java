// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeSubsystemConfigs;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public SparkFlex m_pivotMotor; // change back to SparkFlex
  public SparkFlex m_intakeMotor; // change back to SparkFlex

  public SparkClosedLoopController m_pivotController;
  public SparkClosedLoopController m_intakeController;

  public SparkLimitSwitch m_pivotMotorUpperSwitch;
  public SparkLimitSwitch m_pivotMotorLowerSwitch;

  public IntakeSubsystem() {
    m_pivotMotor = new SparkFlex(IntakeConstants.kPivotMotorCanID, MotorType.kBrushless);
    m_pivotController = m_pivotMotor.getClosedLoopController();
    // m_intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorCanID,
    // MotorType.kBrushless);

    m_pivotMotor.configure(IntakeSubsystemConfigs.pivotMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    // m_intakeMotor.configure(IntakeSubsystemConfigs.intakeMotorConfig,
    // ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);

    m_pivotMotorUpperSwitch = m_pivotMotor.getReverseLimitSwitch();
    m_pivotMotorLowerSwitch = m_pivotMotor.getForwardLimitSwitch();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Pivot Position", m_pivotMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Set Point Position", m_pivotController.getMAXMotionSetpointPosition());
    SmartDashboard.putBoolean("Is it at set point", m_pivotController.isAtSetpoint());
    SmartDashboard.putBoolean("Upper limit switch pressed?", m_pivotMotorUpperSwitch.isPressed());
    SmartDashboard.putBoolean("Lower limit switch pressed?", m_pivotMotorLowerSwitch.isPressed());
    SmartDashboard.putBoolean("Retracted Limit Switch Hit", retractedLimitSwitchHit());
    SmartDashboard.putBoolean("Extended Limit Switch Hit", extendedLimitSwitchHit());

    if (retractedLimitSwitchHit()) {
      m_pivotMotor.getEncoder().setPosition(0);
    }
  }

  // public void home() {
  // m_pivotController.setSetpoint(0.25, ControlType.kDutyCycle);
  // }

  public void set(double dutyCycle) {
    m_pivotController.setSetpoint(
        MathUtil.clamp(dutyCycle, -IntakeConstants.kMaxPivotDutyCycle, IntakeConstants.kMaxPivotDutyCycle),
        ControlType.kDutyCycle);
  }

  public void setAngle(double angleDegrees, ClosedLoopSlot slot) {
    m_pivotController.setSetpoint(angleDegrees, ControlType.kMAXMotionPositionControl, slot, 3.0);
  }

  public boolean retractedLimitSwitchHit() {
    return m_pivotMotorUpperSwitch.isPressed();
  }

  public boolean extendedLimitSwitchHit() {
    return m_pivotMotorLowerSwitch.isPressed();
  }

  public void resetPivotEncoder() {
    m_pivotMotor.getEncoder().setPosition(0);
  }

  public void setPosition(double pos) {
    m_pivotController.setSetpoint(pos, ControlType.kMAXMotionPositionControl);
  }
}
