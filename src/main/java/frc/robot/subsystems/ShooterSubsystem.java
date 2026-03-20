// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShooterSubsystemConfigs;
import frc.robot.Constants.NeoMotorConstants;
// import frc.robot.Constants.NeoVortexConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.data.ShooterDatum;

public class ShooterSubsystem extends SubsystemBase {
	SparkFlex m_topMotor, m_middleMotor, m_bottomMotor;
	// SparkMax m_topMotor, m_middleMotor, m_bottomMotor;
	RelativeEncoder m_topEncoder, m_middleEncoder, m_bottomEncoder;
	SparkFlexConfig m_topConfig, m_middleConfig, m_bottomConfig;
	// SparkMaxConfig m_topConfig, m_middleConfig, m_bottomConfig;
	SparkClosedLoopController m_closedLoopControllerTop;
	SparkClosedLoopController m_closedLoopControllerMiddle;
	SparkClosedLoopController m_closedLoopControllerBottom;
	double m_topSpeed;
	double m_middleSpeed;
	double m_bottomSpeed;
	double m_topMotorP, m_topMotorD, m_topMotorFF;
	double m_middleMotorP, m_middleMotorD, m_middleMotorFF;
	double m_bottomMotorP, m_bottomMotorD, m_bottomMotorFF, m_bottomMotorSFF;
	double m_topMotorPOld, m_topMotorDOld, m_topMotorFFOld;
	double m_middleMotorPOld, m_middleMotorDOld, m_middleMotorFFOld;
	double m_bottomMotorPOld, m_bottomMotorDOld, m_bottomMotorFFOld, m_bottomMotorSFFOld;
	// SparkMaxConfig m_config;

	/** Creates a new ShooterSubsystem. */
	public ShooterSubsystem() {
		m_topMotor = new SparkFlex(ShooterConstants.kTopMotorCanID, MotorType.kBrushless);
		m_middleMotor = new SparkFlex(ShooterConstants.kMiddleMotorCanID, MotorType.kBrushless);
		m_bottomMotor = new SparkFlex(ShooterConstants.kBottomMotorCanID, MotorType.kBrushless);
		// m_topMotor = new SparkMax(ShooterConstants.kTopMotorCanID,
		// MotorType.kBrushless);
		// m_middleMotor = new SparkMax(ShooterConstants.kMiddleMotorCanID,
		// MotorType.kBrushless);
		// m_bottomMotor = new SparkMax(ShooterConstants.kBottomMotorCanID,
		// MotorType.kBrushless);

		m_topMotorP = 0.0004;
		m_topMotorD = 0.02;
		m_topMotorFF = 0.00015;

		// Configuring top motor
		m_topConfig = new SparkFlexConfig();
		// m_topConfig.encoder // change these values
		// .velocityConversionFactor((1.0 / 9.0));
		m_topConfig.encoder.velocityConversionFactor(1.0);
		m_topConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(m_topMotorP, ClosedLoopSlot.kSlot0) // change
				.d(m_topMotorD, ClosedLoopSlot.kSlot0)
				.outputRange(-1, 1);
		// .feedForward.kV(1 / NeoVortexConstants.kMotorkV);
		m_topConfig.smartCurrentLimit(40);
		m_topConfig.inverted(true);
		m_topConfig.closedLoop.feedForward.kV(m_topMotorFF);
		m_topMotor.configure(ShooterSubsystemConfigs.topMotorConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
		// Configuring middle motor

		m_middleMotorP = 0.001;
		m_middleMotorD = 0.001;
		m_middleMotorFF = 0.0002;

		m_middleConfig = new SparkFlexConfig();
		// m_middleConfig = new SparkMaxConfig();
		// m_middleConfig.encoder // change these values
		// .velocityConversionFactor((1.0 / 9.0));
		m_middleConfig.encoder.velocityConversionFactor(1.0);
		m_middleConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(m_middleMotorP, ClosedLoopSlot.kSlot0) // change
				.d(m_middleMotorD, ClosedLoopSlot.kSlot0)
				.outputRange(-1, 1);
		// .feedForward.kV(1 / NeoVortexConstants.kMotorkV);
		m_middleConfig.inverted(false);
		m_middleConfig.smartCurrentLimit(40);
		m_middleConfig.closedLoop.feedForward.kV(m_middleMotorFF);
		m_middleMotor.configure(ShooterSubsystemConfigs.middleMotorConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);

		m_bottomMotor.configure(ShooterSubsystemConfigs.bottomMotorConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);

		m_closedLoopControllerTop = m_topMotor.getClosedLoopController();
		m_closedLoopControllerMiddle = m_middleMotor.getClosedLoopController();
		m_closedLoopControllerBottom = m_bottomMotor.getClosedLoopController();
		m_topEncoder = m_topMotor.getEncoder();
		m_middleEncoder = m_middleMotor.getEncoder();
		m_bottomEncoder = m_bottomMotor.getEncoder();

		SmartDashboard.putNumber("Bottom Motor P", m_bottomMotorP);
		SmartDashboard.putNumber("Bottom Motor D", m_bottomMotorD);
		SmartDashboard.putNumber("Bottom Motor FF", m_bottomMotorFF);
		SmartDashboard.putNumber("Bottom Motor Static FF", m_bottomMotorSFF);
		SmartDashboard.putNumber("Middle Motor P", m_middleMotorP);
		SmartDashboard.putNumber("Middle Motor D", m_middleMotorD);
		SmartDashboard.putNumber("Middle Motor FF", m_middleMotorFF);
		SmartDashboard.putNumber("Top Motor P", m_topMotorP);
		SmartDashboard.putNumber("Top Motor D", m_topMotorD);
		SmartDashboard.putNumber("Top Motor FF", m_topMotorFF);
	}

	public void voltageToRPM(double voltage) {

	}

	public void moveTopMotor(double PWM) {
		// m_topMotor.set(PWM);
		m_closedLoopControllerTop.setReference(PWM, ControlType.kVelocity,
				ClosedLoopSlot.kSlot1);
	}

	public void moveMiddleMotor(double PWM) {
		m_middleMotor.set(PWM);
		// m_closedLoopControllerRight.setReference(RPM, ControlType.kVelocity,
		// ClosedLoopSlot.kSlot1);
	}

	public void moveBottomMotor(double PWM) {
		m_bottomMotor.set(PWM);
		// m_closedLoopControllerRight.setReference(RPM, ControlType.kVelocity,
		// ClosedLoopSlot.kSlot1);
	}

	public void setSpeedTop(double speed) {
		// m_topSpeed = speed / (7.61 * 0.65);
		m_topSpeed = speed;
		m_closedLoopControllerTop.setSetpoint(m_topSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
		// m_closedLoopControllerTop.setSetpoint(m_topSpeed /
		// NeoVortexConstants.kMotorkV, ControlType.kVoltage);
	}

	public void setSpeedTopToTarget() {
		setSpeedTop(m_topSpeed);
	}

	public void setSpeedMiddle(double speed) {
		m_middleSpeed = speed;
		m_closedLoopControllerMiddle.setSetpoint(m_middleSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
		// m_closedLoopControllerMiddle.setSetpoint(m_middleSpeed /
		// NeoVortexConstants.kMotorkV, ControlType.kVoltage);
	}

	public void setSpeedMiddleToTarget() {
		setSpeedTop(m_middleSpeed);
	}

	public void setSpeedBottom(double speed) {
		m_bottomSpeed = speed;
		m_closedLoopControllerBottom.setSetpoint(m_bottomSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
		// m_closedLoopControllerBottom.setSetpoint(m_bottomSpeed /
		// NeoVortexConstants.kMotorkV, ControlType.kVoltage);
	}

	public void setSpeedBottomToTarget() {
		setSpeedBottom(m_bottomSpeed);
	}

	// internally uses the voltage contol loop instead of velocity pid
	// uses the motor's Kv rating to convert RPMs to Volts
	public void setSpeedTopWithVoltage(double speed) {
		m_closedLoopControllerTop.setSetpoint(speed / NeoMotorConstants.kKV, ControlType.kVoltage,
				ClosedLoopSlot.kSlot2);
	}

	// internally uses the voltage contol loop instead of velocity pid
	// uses the motor's Kv rating to convert RPMs to Volts
	public void setSpeedMiddleWithVoltage(double speed) {
		m_closedLoopControllerMiddle.setSetpoint(speed / NeoMotorConstants.kKV, ControlType.kVoltage,
				ClosedLoopSlot.kSlot2);
	}

	// internally uses the voltage contol loop instead of velocity pid
	// uses the motor's Kv rating to convert RPMs to Volts
	public void setSpeedBottomWithVoltage(double speed) {
		m_closedLoopControllerBottom.setSetpoint(speed / NeoMotorConstants.kKV, ControlType.kVoltage,
				ClosedLoopSlot.kSlot2);
	}

	// in rpm
	public double getTopMotorSpeed() {
		return m_topEncoder.getVelocity();
	}

	// in rpm
	public double getMiddleMotorSpeed() {
		return m_middleEncoder.getVelocity();
	}

	// in rpm
	public double getBottomMotorSpeed() {
		return m_bottomEncoder.getVelocity();
	}

	public double getTopMotorSpeedTarget() {
		return m_topSpeed;
	}

	public double getMiddleMotorSpeedTarget() {
		return m_middleSpeed;
	}

	public double getBottomMotorSpeedTarget() {
		return m_bottomSpeed;
	}

	public void SetShooterTargetDistance(double targetDistance) {
		double lowKey = 0;
		double highKey = 0;
		for (Map.Entry<Double, ShooterDatum> entry : ShooterConstants.kShooterDistanceToRPMsMap.entrySet()) {
			double key = entry.getKey();
			if (targetDistance < key) {
				highKey = key;
				break;
			} else {
				lowKey = key;
			}
		}

		ShooterDatum lowVal = ShooterConstants.kShooterDistanceToRPMsMap.get(lowKey);
		ShooterDatum highVal = ShooterConstants.kShooterDistanceToRPMsMap.get(highKey);
		// Math

		m_topSpeed = Lerp(lowKey, highKey, lowVal.m_topSpeed, highVal.m_topSpeed, targetDistance);
		m_middleSpeed = Lerp(lowKey, highKey, lowVal.m_middleSpeed, highVal.m_middleSpeed, targetDistance);
		m_bottomSpeed = Lerp(lowKey, highKey, lowVal.m_bottomSpeed, highVal.m_bottomSpeed, targetDistance);

		// never lets bottom speed be greater than top speed
		m_bottomSpeed = MathUtil.clamp(m_bottomSpeed, 0, m_topSpeed);
	}

	public double Lerp(double lowKey, double highKey, double lowRPM, double highRPM, double distance) {
		double percent = (distance - lowKey) / (highKey - lowKey);
		return (percent) * (highRPM - lowRPM) + lowRPM;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Top Motor Speed", getTopMotorSpeed());
		SmartDashboard.putNumber("Middle Motor Speed", getMiddleMotorSpeed());
		SmartDashboard.putNumber("Bottom Motor Speed", getBottomMotorSpeed());
		m_bottomMotorP = SmartDashboard.getNumber("Bottom Motor P", m_bottomMotorPOld);
		m_bottomMotorD = SmartDashboard.getNumber("Bottom Motor D", m_bottomMotorDOld);
		m_bottomMotorFF = SmartDashboard.getNumber("Bottom Motor FF", m_bottomMotorFFOld);
		m_bottomMotorSFF = SmartDashboard.getNumber("Bottom Motor Static FF", m_bottomMotorSFFOld);
		m_middleMotorP = SmartDashboard.getNumber("Middle Motor P", m_middleMotorPOld);
		m_middleMotorD = SmartDashboard.getNumber("Middle Motor D", m_middleMotorDOld);
		m_middleMotorFF = SmartDashboard.getNumber("Middle Motor FF", m_middleMotorFFOld);
		m_topMotorP = SmartDashboard.getNumber("Top Motor P", m_topMotorPOld);
		m_topMotorD = SmartDashboard.getNumber("Top Motor D", m_topMotorDOld);
		m_topMotorFF = SmartDashboard.getNumber("Top Motor FF", m_topMotorFFOld);

		if (m_topMotorP != m_topMotorPOld || m_topMotorD != m_topMotorDOld ||
				m_topMotorFF != m_topMotorFFOld) {
			m_topConfig.closedLoop.p(m_topMotorP).d(m_topMotorD).feedForward.kV(m_topMotorFF);
			m_topMotor.configure(m_topConfig, ResetMode.kResetSafeParameters,
					PersistMode.kPersistParameters);
			m_topMotorPOld = m_topMotorP;
			m_topMotorDOld = m_topMotorD;
			m_topMotorFFOld = m_topMotorFF;
			System.out.println("reconfiguring top motor");
		}
		if (m_middleMotorP != m_middleMotorPOld || m_middleMotorD != m_middleMotorDOld
				|| m_middleMotorFF != m_middleMotorFFOld) {
			m_middleConfig.closedLoop.p(m_middleMotorP).d(m_middleMotorD).feedForward.kV(m_middleMotorFF);
			m_middleMotor.configure(m_middleConfig, ResetMode.kResetSafeParameters,
					PersistMode.kPersistParameters);
			m_middleMotorPOld = m_middleMotorP;
			m_middleMotorDOld = m_middleMotorD;
			m_middleMotorFFOld = m_middleMotorFF;
		}
		if (m_bottomMotorP != m_bottomMotorPOld || m_bottomMotorD != m_bottomMotorDOld
				|| m_bottomMotorFF != m_bottomMotorFFOld || m_bottomMotorSFF != m_bottomMotorSFFOld) {
			m_bottomConfig.closedLoop.p(m_bottomMotorP).d(m_bottomMotorD).feedForward.kV(m_bottomMotorFF)
					.kS(m_bottomMotorSFF);
			m_bottomMotor.configure(m_bottomConfig, ResetMode.kResetSafeParameters,
					PersistMode.kPersistParameters);
			m_bottomMotorPOld = m_bottomMotorP;
			m_bottomMotorDOld = m_bottomMotorD;
			m_bottomMotorFFOld = m_bottomMotorFF;
			m_bottomMotorSFFOld = m_bottomMotorSFF;
		}

		// double topSpeed = m_topController.calculate(getTopMotorSpeed());
		// // System.out.println(topSpeed);
		// m_closedLoopControllerTop.setSetpoint(
		// topSpeed / NeoVortexConstants.kMotorkV, ControlType.kVoltage);

		// double middleSpeed = m_middleController.calculate(getMiddleMotorSpeed());
		// // System.out.println(middleSpeed);
		// m_closedLoopControllerMiddle.setSetpoint(
		// middleSpeed / NeoVortexConstants.kMotorkV,
		// ControlType.kVoltage);

		// double bottomSpeed = m_bottomController.calculate(getBottomMotorSpeed());
		// // System.out.println(bottomSpeed);
		// m_closedLoopControllerBottom.setSetpoint(
		// bottomSpeed / NeoVortexConstants.kMotorkV,
		// ControlType.kVoltage);

		// This method will be called once per scheduler run
	}
}
