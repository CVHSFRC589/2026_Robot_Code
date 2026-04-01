package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;

public final class Configs {
	public static final class MAXSwerveModule {
		public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
		public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

		static {
			// Use module constants to calculate conversion factors and feed forward gain.
			double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
					/ ModuleConstants.kDrivingMotorReduction;
			double turningFactor = 2 * Math.PI;
			double nominalVoltage = 12.0;
			double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

			drivingConfig
					.idleMode(IdleMode.kBrake)
					.smartCurrentLimit(50);
			drivingConfig.encoder
					.positionConversionFactor(drivingFactor) // meters
					.velocityConversionFactor(drivingFactor / 60.0); // meters per second
			drivingConfig.closedLoop

					.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
					// These are example gains you may need to them for your own robot!
					.pid(0.04, 0, 0)
					.outputRange(-1, 1).feedForward.kV(drivingVelocityFeedForward);

			turningConfig
					.idleMode(IdleMode.kBrake)
					.smartCurrentLimit(20);

			turningConfig.absoluteEncoder
					// Invert the turning encoder, since the output shaft rotates in the opposite
					// direction of the steering motor in the MAXSwerve Module.
					.inverted(true)
					.positionConversionFactor(turningFactor) // radians
					.velocityConversionFactor(turningFactor / 60.0) // radians per second
					// This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for
					// V1):
					.apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

			turningConfig.closedLoop
					.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
					// These are example gains you may need to them for your own robot!
					.pid(1, 0, 0)
					.outputRange(-1, 1)
					// Enable PID wrap around for the turning motor. This will allow the PID
					// controller to go through 0 to get to the setpoint i.e. going from 350 degrees
					// to 10 degrees will go through 0 rather than the other direction which is a
					// longer route.
					.positionWrappingEnabled(true)
					.positionWrappingInputRange(0, turningFactor);
		}
	}

	public static final class IntakeSubsystemConfigs {
		public static final SparkFlexConfig pivotMotorConfig = new SparkFlexConfig();
		public static final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();
		static {
			pivotMotorConfig
					.idleMode(IdleMode.kBrake)
					.smartCurrentLimit(30);

			pivotMotorConfig.encoder
					.positionConversionFactor(IntakeConstants.kPivotMotorPositionConversionFactor)
					.velocityConversionFactor(IntakeConstants.kPivotMotorVelocityConversionFactor);

			pivotMotorConfig.closedLoop
					.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
					.pid(0.02, 0, 0, ClosedLoopSlot.kSlot0)
					.outputRange(-1, 1).maxMotion
					.cruiseVelocity(Constants.IntakeConstants.kCruiseVel0, ClosedLoopSlot.kSlot0)
					.maxAcceleration(Constants.IntakeConstants.kMaxAccel0, ClosedLoopSlot.kSlot0)
					.allowedProfileError(Constants.IntakeConstants.kAllowedProfileError0, ClosedLoopSlot.kSlot0);
			// .cruiseVelocity(Constants.IntakeConstants.kCruiseVel1, ClosedLoopSlot.kSlot1)
			// .maxAcceleration(Constants.IntakeConstants.kMaxAccel1, ClosedLoopSlot.kSlot1)
			// .allowedProfileError(Constants.IntakeConstants.kAllowedProfileError1,
			// ClosedLoopSlot.kSlot1);
			// pivotMotorConfig.closedLoop.feedForward.
			// .kS(Constants.IntakeConstants.kPivotS, ClosedLoopSlot.kSlot0) // make sure to
			// change s value
			// .kV(Constants.IntakeConstants.kPivotV, ClosedLoopSlot.kSlot0) // slot 0
			// explicitly
			// .kA(Constants.IntakeConstants.kPivotA, ClosedLoopSlot.kSlot0)
			// //.kCos(Constants.IntakeConstants.kPivotCos, ClosedLoopSlot.kSlot0)
			// .kCosRatio(Constants.IntakeConstants.kPivotCosRatio, ClosedLoopSlot.kSlot0)
			// .kS(Constants.IntakeConstants.kPivotS, ClosedLoopSlot.kSlot1) // make sure to
			// change s value
			// .kV(Constants.IntakeConstants.kPivotV, ClosedLoopSlot.kSlot1) // slot 0
			// explicitly
			// .kA(Constants.IntakeConstants.kPivotA, ClosedLoopSlot.kSlot1)
			// .kCos(Constants.IntakeConstants.kPivotCos, ClosedLoopSlot.kSlot1)
			// .kCosRatio(Constants.IntakeConstants.kPivotCosRatio, ClosedLoopSlot.kSlot1);

			// .feedForward.kV(drivingVelocityFeedForward);

			intakeMotorConfig
					.idleMode(IdleMode.kCoast)
					.smartCurrentLimit(30)
					.inverted(true);

			intakeMotorConfig.encoder
					.positionConversionFactor(IntakeConstants.kIntakeMotorPositionConversionFactor)
					.velocityConversionFactor(IntakeConstants.kIntakeMotorVelocityConversionFactor);

			intakeMotorConfig.closedLoop
					.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
					// These are example gains you may need to them for your own robot!
					.p(0.0125)
					.outputRange(-1, 1)
					.d(0.05).feedForward.kS(8).kV(0.05);
			intakeMotorConfig.encoder
					.uvwMeasurementPeriod(8)
					.quadratureAverageDepth(2)
					.quadratureMeasurementPeriod(8);

			// .feedForward.kV(drivingVelocityFeedForward);
		}
	}

	public static final class ClimberSubsystemConfigs {
		public static final SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
		public static final SparkFlexConfig rightMotorConfig = new SparkFlexConfig();

		static {
			leftMotorConfig
					.idleMode(IdleMode.kBrake)
					.smartCurrentLimit(15)
					.inverted(true);

			leftMotorConfig.encoder
					.positionConversionFactor(ClimberConstants.kMotorPositionConversionFactor)
					.velocityConversionFactor(ClimberConstants.kMotorVelocityConversionFactor);

			leftMotorConfig.closedLoop
					.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
					// These are example gains you may need to them for your own robot!
					.pid(ClimberConstants.kMotorP, ClimberConstants.kMotorI, 0)
					.outputRange(-1, 1);
			// .feedForward.kV(drivingVelocityFeedForward);

			leftMotorConfig.closedLoop.maxMotion
					.cruiseVelocity(ClimberConstants.kMaxExtendVelocity, ClosedLoopSlot.kSlot0)
					.cruiseVelocity(ClimberConstants.kMaxRetractVelocity, ClosedLoopSlot.kSlot1)
					.maxAcceleration(ClimberConstants.kMaxExtendAcceleration, ClosedLoopSlot.kSlot0)
					.maxAcceleration(ClimberConstants.kMaxRetractAcceleration, ClosedLoopSlot.kSlot1);

			rightMotorConfig
					.idleMode(IdleMode.kBrake)
					.smartCurrentLimit(15);

			rightMotorConfig.encoder
					.positionConversionFactor(ClimberConstants.kMotorPositionConversionFactor)
					.velocityConversionFactor(ClimberConstants.kMotorVelocityConversionFactor);

			rightMotorConfig.closedLoop
					.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
					// These are example gains you may need to them for your own robot!
					.pid(ClimberConstants.kMotorP, ClimberConstants.kMotorI, 0)
					.outputRange(-1, 1);
			// .feedForward.kV(drivingVelocityFeedForward);

			rightMotorConfig.closedLoop.maxMotion
					.cruiseVelocity(ClimberConstants.kMaxExtendVelocity, ClosedLoopSlot.kSlot0)
					.cruiseVelocity(ClimberConstants.kMaxRetractVelocity, ClosedLoopSlot.kSlot1)
					.maxAcceleration(ClimberConstants.kMaxExtendAcceleration, ClosedLoopSlot.kSlot0)
					.maxAcceleration(ClimberConstants.kMaxRetractAcceleration, ClosedLoopSlot.kSlot1);
		}
	}

	public static final class ShooterSubsystemConfigs {
		public static final SparkFlexConfig topMotorConfig = new SparkFlexConfig();
		public static final SparkFlexConfig middleMotorConfig = new SparkFlexConfig();
		public static final SparkFlexConfig bottomMotorConfig = new SparkFlexConfig();

		static {
			// m_topConfig.encoder // change these values
			// .velocityConversionFactor((1.0 / 9.0));
			topMotorConfig.encoder.velocityConversionFactor(1.0);
			topMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
					.p(ShooterConstants.kTopMotorP, ClosedLoopSlot.kSlot0) // change
					.d(ShooterConstants.kTopMotorD, ClosedLoopSlot.kSlot0)
					.outputRange(-1, 1);
			// .feedForward.kV(1 / NeoVortexConstants.kMotorkV);
			topMotorConfig.smartCurrentLimit(40);
			topMotorConfig.inverted(true);
			topMotorConfig.closedLoop.feedForward.kV(ShooterConstants.kTopMotorFF);
			topMotorConfig.encoder
					.uvwMeasurementPeriod(8)
					.quadratureAverageDepth(2)
					.quadratureMeasurementPeriod(8);

			// m_middleConfig = new SparkMaxConfig();
			// m_middleConfig.encoder // change these values
			// .velocityConversionFactor((1.0 / 9.0));
			middleMotorConfig.encoder.velocityConversionFactor(1.0);
			middleMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
					.p(ShooterConstants.kMiddleMotorP, ClosedLoopSlot.kSlot0) // change
					.d(ShooterConstants.kMiddleMotorD, ClosedLoopSlot.kSlot0)
					.outputRange(-1, 1);
			// .feedForward.kV(1 / NeoVortexConstants.kMotorkV);
			middleMotorConfig.inverted(false);
			middleMotorConfig.smartCurrentLimit(100);
			middleMotorConfig.closedLoop.feedForward.kV(ShooterConstants.kMiddleMotorFF);
			middleMotorConfig.encoder
					.uvwMeasurementPeriod(8)
					.quadratureAverageDepth(2)
					.quadratureMeasurementPeriod(8);

			// m_bottomConfig = new SparkMaxConfig();
			// m_bottomConfig.encoder // change these values
			// .velocityConversionFactor((1.0 / 9.0));
			bottomMotorConfig.encoder.velocityConversionFactor(1.0 / 5.0);
			bottomMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
					.p(ShooterConstants.kBottomMotorP, ClosedLoopSlot.kSlot0) // change
					.i(ShooterConstants.kBottomMotorI, ClosedLoopSlot.kSlot0)
					.d(ShooterConstants.kBottomMotorD, ClosedLoopSlot.kSlot0)
					.outputRange(-1, 1);
			// .feedForward.kV(1 / NeoVortexConstants.kMotorkV);
			bottomMotorConfig.inverted(true);
			bottomMotorConfig.closedLoop.feedForward.kV(ShooterConstants.kBottomMotorFF)
					.kS(ShooterConstants.kBottomMotorSFF);
			bottomMotorConfig.smartCurrentLimit(80);
			bottomMotorConfig.encoder
					.uvwMeasurementPeriod(8)
					.quadratureAverageDepth(2)
					.quadratureMeasurementPeriod(8);
		}
	}
}
