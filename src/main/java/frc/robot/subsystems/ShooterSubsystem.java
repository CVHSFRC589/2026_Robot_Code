// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.NeoVortexConstants;
import frc.robot.Constants.ShooterConstants;

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
	double m_topMotorP, m_topMotorI, m_topMotorIZone;
	double m_middleMotorP, m_middleMotorI, m_middleMotorIZone;
	double m_bottomMotorP, m_bottomMotorI, m_bottomMotorIZone;
	double m_topMotorPOld, m_topMotorIOld, m_topMotorIZoneOld;
	double m_middleMotorPOld, m_middleMotorIOld, m_middleMotorIZoneOld;
	double m_bottomMotorPOld, m_bottomMotorIOld, m_bottomMotorIZoneOld;
	PIDController m_topController, m_middleController, m_bottomController;
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

		// Configuring top motor
		m_topConfig = new SparkFlexConfig();
		// m_topConfig.encoder // change these values
		// .velocityConversionFactor((1.0 / 9.0));
		m_topConfig.encoder.velocityConversionFactor(1.0);
		m_topConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(0.0002, ClosedLoopSlot.kSlot0) // change
				.i(0.000001, ClosedLoopSlot.kSlot0)
				.d(0, ClosedLoopSlot.kSlot0)
				.outputRange(-1, 1);
		// .feedForward.kV(1 / NeoVortexConstants.kMotorkV);
		m_topConfig.smartCurrentLimit(50);
		m_topConfig.inverted(true);
		m_topMotor.configure(m_topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configuring middle motor
		m_middleConfig = new SparkFlexConfig();
		// m_middleConfig = new SparkMaxConfig();
		// m_middleConfig.encoder // change these values
		// .velocityConversionFactor((1.0 / 9.0));
		m_middleConfig.encoder.velocityConversionFactor(1.0);
		m_middleConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(0.0002, ClosedLoopSlot.kSlot0) // change
				.i(0.000001, ClosedLoopSlot.kSlot0)
				.d(0, ClosedLoopSlot.kSlot0)
				.outputRange(-1, 1);
		// .feedForward.kV(1 / NeoVortexConstants.kMotorkV);
		m_middleConfig.inverted(false);
		m_middleMotor.configure(m_middleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Configuring bottom motor
		m_bottomConfig = new SparkFlexConfig();
		// m_bottomConfig = new SparkMaxConfig();
		// m_bottomConfig.encoder // change these values
		// .velocityConversionFactor((1.0 / 9.0));
		m_bottomConfig.encoder.velocityConversionFactor(1.0);
		m_bottomConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(0.0002, ClosedLoopSlot.kSlot0) // change
				.i(0.000001, ClosedLoopSlot.kSlot0)
				.d(0.05, ClosedLoopSlot.kSlot0)
				.outputRange(-1, 1);
		// .feedForward.kV(1 / NeoVortexConstants.kMotorkV);
		m_bottomConfig.inverted(true);
		m_bottomMotor.configure(m_bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		m_closedLoopControllerTop = m_topMotor.getClosedLoopController();
		m_closedLoopControllerMiddle = m_middleMotor.getClosedLoopController();
		m_closedLoopControllerBottom = m_bottomMotor.getClosedLoopController();
		m_topEncoder = m_topMotor.getEncoder();
		m_middleEncoder = m_middleMotor.getEncoder();
		m_bottomEncoder = m_bottomMotor.getEncoder();
		m_topController = new PIDController(1, 0.1, 0);
		m_middleController = new PIDController(1, 0.1, 0);
		m_bottomController = new PIDController(1, 0.1, 0);

		SmartDashboard.putNumber("Bottom Motor P", m_bottomMotorP);
		SmartDashboard.putNumber("Bottom Motor I", m_bottomMotorI);
		SmartDashboard.putNumber("Bottom Motor I Zone", m_bottomMotorIZone);
		SmartDashboard.putNumber("Middle Motor P", m_middleMotorP);
		SmartDashboard.putNumber("Middle Motor I", m_middleMotorI);
		SmartDashboard.putNumber("Middle Motor I Zone", m_middleMotorIZone);
		SmartDashboard.putNumber("Top Motor P", m_topMotorP);
		SmartDashboard.putNumber("Top Motor I", m_topMotorI);
		SmartDashboard.putNumber("Top Motor I Zone", m_topMotorIZone);
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

	public void setSpeedMiddle(double speed) {
		m_middleSpeed = speed;
		m_closedLoopControllerMiddle.setSetpoint(m_middleSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
		// m_closedLoopControllerMiddle.setSetpoint(m_middleSpeed /
		// NeoVortexConstants.kMotorkV, ControlType.kVoltage);
	}

	public void setSpeedBottom(double speed) {
		m_bottomSpeed = speed;
		m_closedLoopControllerBottom.setSetpoint(m_bottomSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
		// m_closedLoopControllerBottom.setSetpoint(m_bottomSpeed /
		// NeoVortexConstants.kMotorkV, ControlType.kVoltage);
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

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Top Motor Speed", getTopMotorSpeed());
		SmartDashboard.putNumber("Middle Motor Speed", getMiddleMotorSpeed());
		SmartDashboard.putNumber("Bottom Motor Speed", getBottomMotorSpeed());
		m_bottomMotorP = SmartDashboard.getNumber("Bottom Motor P", m_bottomMotorPOld);
		m_bottomMotorI = SmartDashboard.getNumber("Bottom Motor I", m_bottomMotorIOld);
		m_bottomMotorIZone = SmartDashboard.getNumber("Bottom Motor I Zone", m_bottomMotorIZoneOld);
		m_middleMotorP = SmartDashboard.getNumber("Middle Motor P", m_middleMotorPOld);
		m_middleMotorI = SmartDashboard.getNumber("Middle Motor I", m_middleMotorIOld);
		m_middleMotorIZone = SmartDashboard.getNumber("Middle Motor I Zone", m_middleMotorIZoneOld);
		m_topMotorP = SmartDashboard.getNumber("Top Motor P", m_topMotorPOld);
		m_topMotorI = SmartDashboard.getNumber("Top Motor I", m_topMotorIOld);
		m_topMotorIZone = SmartDashboard.getNumber("Top Motor I Zone", m_topMotorIZoneOld);

		// if (m_topMotorP != m_topMotorPOld || m_topMotorI != m_topMotorIOld ||
		// m_topMotorIZone != m_topMotorIZoneOld) {
		// m_topConfig.closedLoop.p(m_topMotorP).i(m_topMotorI).iZone(m_topMotorIZone);
		// m_topMotor.configure(m_topConfig, ResetMode.kResetSafeParameters,
		// PersistMode.kPersistParameters);
		// m_topMotorPOld = m_topMotorP;
		// m_topMotorIOld = m_topMotorI;
		// m_topMotorIZoneOld = m_topMotorIZone;
		// System.out.println("reconfiguring top motor");
		// }
		// if (m_middleMotorP != m_middleMotorPOld || m_middleMotorI !=
		// m_middleMotorIOld
		// || m_middleMotorIZone != m_middleMotorIZoneOld) {
		// m_middleConfig.closedLoop.p(m_middleMotorP).i(m_middleMotorI).iZone(m_middleMotorIZone);
		// m_middleMotor.configure(m_middleConfig, ResetMode.kResetSafeParameters,
		// PersistMode.kPersistParameters);
		// m_middleMotorPOld = m_middleMotorP;
		// m_middleMotorIOld = m_middleMotorI;
		// m_middleMotorIZoneOld = m_middleMotorIZone;
		// }
		// if (m_bottomMotorP != m_bottomMotorPOld || m_bottomMotorI != m_bottomMotorIOld
		// 		|| m_bottomMotorIZone != m_bottomMotorIZoneOld) {
		// 	m_bottomConfig.closedLoop.p(m_bottomMotorP).i(m_bottomMotorI).d(m_bottomMotorIZone);
		// 	m_bottomMotor.configure(m_bottomConfig, ResetMode.kResetSafeParameters,
		// 			PersistMode.kPersistParameters);
		// 	m_bottomMotorPOld = m_bottomMotorP;
		// 	m_bottomMotorIOld = m_bottomMotorI;
		// 	m_bottomMotorIZoneOld = m_bottomMotorIZone;
		// }

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
