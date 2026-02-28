// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ClimberSubsystemConfigs;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public Climber m_leftClimber, m_rightClimber;

  public ClimberSubsystem() {
    m_leftClimber = new Climber(ClimberConstants.kLeftMotorCanID, ClimberSubsystemConfigs.leftMotorConfig);
    m_rightClimber = new Climber(ClimberConstants.kRightMotorCanID, ClimberSubsystemConfigs.rightMotorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climber Position", m_leftClimber.getPosition());
    SmartDashboard.putNumber("Right Climber Position", m_rightClimber.getPosition());

    if (m_leftClimber.isLimitSwitchTouched()) {
      m_leftClimber.resetEncoder();
    }
    if (m_rightClimber.isLimitSwitchTouched()) {
      m_rightClimber.resetEncoder();
    }
  }

  public void setLeftClimberPosition(double position) {
    m_leftClimber.m_controller.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setRightClimberPosition(double position) {
    m_rightClimber.m_controller.setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public class Climber {
    public SparkMax m_motor;
    public SparkClosedLoopController m_controller;
    public SparkLimitSwitch m_limitSwitch;

    public Climber(int CanID, SparkBaseConfig motorConfig) {
      m_motor = new SparkMax(CanID, MotorType.kBrushless);
      m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_limitSwitch = m_motor.getForwardLimitSwitch();

      m_controller = m_motor.getClosedLoopController();
    }

    public void setSpeed(double speed) {
      m_controller.setSetpoint(speed, ControlType.kVelocity);
    }

    public void set(double speed) {
      m_motor.set(MathUtil.clamp(speed, -ClimberConstants.kMaxSpeed, ClimberConstants.kMaxSpeed));
    }

    public void setPosition(double pos) {
      m_controller.setSetpoint(pos, ControlType.kMAXMotionPositionControl);
    }

    public boolean isLimitSwitchTouched() {
      return m_limitSwitch.isPressed();
    }

    public void resetEncoder() {
      m_motor.getEncoder().setPosition(0);
    }

    public double getPosition() {
      return m_motor.getEncoder().getPosition();
    }

  }
}
