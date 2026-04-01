// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.data.ShooterDatum;

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
	public static final class DriveConstants {
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		public static final double kMaxSpeedMetersPerSecond = 4.8; // old is 4.8
		public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second // old is 2 * Math.PI

		// Chassis configuration
		// public static final double kTrackWidth = Units.inchesToMeters(26.5);
		public static final double kTrackWidth = Units.inchesToMeters(23.5);
		// Distance between centers of right and left wheels on robot
		// public static final double kWheelBase = Units.inchesToMeters(26.5);
		public static final double kWheelBase = Units.inchesToMeters(23.5);
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
		public static final int kFrontLeftDrivingCanId = 10;
		public static final int kRearLeftDrivingCanId = 30;
		public static final int kFrontRightDrivingCanId = 20;
		public static final int kRearRightDrivingCanId = 40;

		public static final int kFrontLeftTurningCanId = 11;
		public static final int kRearLeftTurningCanId = 31;
		public static final int kFrontRightTurningCanId = 21;
		public static final int kRearRightTurningCanId = 41;

		public static final boolean kGyroReversed = true;
		public static final int kPigeonCanID = 60;

		// PID
		public static final double kRotationalPIDkP = 0.015;
		public static final double kRotationalPIDkI = 1e-5;
		public static final double kRotationalPIDkD = 0.0001;
		public static final double kRotationalDeadband = 0.02;
	}

	public static final class IntakeConstants {
		public static final int kPivotMotorCanID = 58;
		public static final int kIntakeMotorCanID = 59;

		public static final double kPivotMotorPositionConversionFactor = 360.0 / 50.0;
		public static final double kPivotMotorVelocityConversionFactor = 360.0 / 50.0;
		public static final double kIntakeMotorPositionConversionFactor = 1.0;
		public static final double kIntakeMotorVelocityConversionFactor = 1.0;

		public static final double kIntakeExtendedPosition = 160;
		public static final double kIntakeRetractedPosition = 0;

		public static final double kMaxPivotDutyCycle = 0.25;

		public static final double kCruiseVel0 = 1000;
		public static final double kMaxAccel0 = 500;
		public static final double kAllowedProfileError0 = kCruiseVel0 * 0.1;

		public static final double kCruiseVel1 = 10;
		public static final double kMaxAccel1 = 50;
		public static final double kAllowedProfileError1 = kCruiseVel1 * 0.1;

		public static final double kPivotS = 0;
		public static final double kPivotV = 5.27;
		public static final double kPivotA = 0.06;
		public static final double kPivotCos = 3; // old = 0.15
		public static final double kPivotCosRatio = 1.0; // Either 50 or 1/50

		public static final double kIntakeFullSpeed = 5000; // in rpm
	}

	public static final class ClimberConstants {
		public static final int kLeftMotorCanID = 55;
		public static final int kRightMotorCanID = 56;
		public static final double kMaxSpeed = 0.1;

		public static final double kMotorP = 0.1;
		public static final double kMotorI = 0.0;

		public static final double kMaxExtendDistance = 24.0;
		public static final double kMaxRetractDistance = 1;
		public static final double kHomeDutyCycle = -0.2; // keep this value small (0 is no power, 1 is full power)

		public static final double kMaxExtendVelocity = 0.15;
		public static final double kMaxRetractVelocity = 0.15;
		public static final double kMaxExtendAcceleration = 0.1;
		public static final double kMaxRetractAcceleration = 0.1;

		public static final int kPinionGearTeeth = 15;
		public static final double kPinionGearDP = 10;
		public static final double kPinionGearRadius = kPinionGearTeeth / (2 * kPinionGearDP);
		public static final double kGearboxReduction = 1 / 100;
		// public static final double kMotorPositionConversionFactor = kGearboxReduction
		// * kPinionGearRadius;
		public static final double kMotorPositionConversionFactor = 12.0 / 100.0;
		// public static final double kMotorVelocityConversionFactor = kGearboxReduction
		// * kPinionGearRadius;
		public static final double kMotorVelocityConversionFactor = 12.0 / 100.0;
	}

	public static final class ShooterConstants {
		public static final int kBottomMotorCanID = 50;
		public static final int kMiddleMotorCanID = 51;
		public static final int kTopMotorCanID = 52;

		public static final double voltageToRPMRatio = ((-542.978271484375 / -1.0) + (-2768.88671875 / -5.0)
				+ (-1657.4326171875 / -3.0)) / 3.0;

		public static final Map<Double, ShooterDatum> kShooterDistanceToRPMsMap = new HashMap<>();

		public static final double kTopMotorP = 0.0001;
		public static final double kTopMotorD = 0.000005;
		public static final double kTopMotorFF = 0.002;

		public static final double kMiddleMotorP = 0.001;
		public static final double kMiddleMotorD = 0.00005;
		public static final double kMiddleMotorFF = 0.001;

		public static final double kBottomMotorP = 0.015; // 0.01
		public static final double kBottomMotorI = 0.00000; // 0.00000
		public static final double kBottomMotorD = 0.002; // 0.01
		public static final double kBottomMotorFF = 0.001; // 0.001
		public static final double kBottomMotorSFF = 5; // 5

		public static final double kTopMotorSpinUpSpeed = 0;// 2750 + (2570 * 0.3);
		public static final double kMiddleMotorSpinUpSpeed = 3000;// 1250 + (1250 * 0.1);
		public static final double kBottomMotorSpinUpSpeed = 600; // 1250

		public static void LoadShooterToRpmMap() {
			// put all tested values for rpms's
			// distance in meters
			kShooterDistanceToRPMsMap.put(2.0, new ShooterDatum(2000, 1000, 2000));

			// kShooterDistanceToRPMsMap.entrySet()
		}
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
		public static final double kDriveDeadband = 0.05;
	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxAccelerationMetersPerSecondSquared = 3;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

		public static final double kPTransController = 0.25; // 4 is ok but not good
		// public static final double kPYController = 1;
		public static final double kPThetaController = 0.625;

		// Constraint for the motion profiled robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
	}

	public static final class NeoMotorConstants {
		public static final double kFreeSpeedRpm = 6784;
		public static final double kKV = 565;
	}

	public static final class CameraConstants {
		public static final Transform3d kRobotToFrontCam = new Transform3d( // seems to be incorrect in sign or
																			// something similar
				new Translation3d(Units.inchesToMeters(11.090), Units.inchesToMeters(8.75),
						Units.inchesToMeters(19.7204)),
				new Rotation3d(0, Units.degreesToRadians(32), 0));
		public static final Transform3d kRobotToBackCam = new Transform3d( // seems to be more correct than the forward
																			// camera
				new Translation3d(Units.inchesToMeters(-11.0697), Units.inchesToMeters(-11.125),
						Units.inchesToMeters(17.467)),
				new Rotation3d(0, Units.degreesToRadians(160), 0));
		public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, 1); // old: 4, 4, 8
		public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
	}

	public static final class FieldConstants {

		// FIELD POSITION CONSTANTS
		public static final Pose2d kRedHubPose = new Pose2d(Units.inchesToMeters(469.44), Units.inchesToMeters(158.84),
				new Rotation2d(0));
		public static final Pose2d kBlueHubPose = new Pose2d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.84),
				new Rotation2d(0));
		public static final Pose2d kRedTrenchLeftPose = new Pose2d(Units.inchesToMeters(470.59),
				Units.inchesToMeters(25.37), new Rotation2d(0));

		// implement change per color side
		// add true points for passing
		public static final Pose2d kPassingPointBlueLeftPose = new Pose2d();
		public static final Pose2d kPassingPointBlueRightPose = new Pose2d();
		public static final Pose2d kPassingPointRedLeftPose = new Pose2d();
		public static final Pose2d kPassingPointRedRightPose = new Pose2d();

		public static final AprilTagFieldLayout kTagLayoutComp = AprilTagFieldLayout
				.loadField(AprilTagFields.kDefaultField);

		public static AprilTagFieldLayout LoadLayout(boolean isHome) {
			if (isHome) {
				try {
					return new AprilTagFieldLayout(
							new File(Filesystem.getDeployDirectory(), "fields/april_tag_layouts/home.json").getPath());
				} catch (IOException e) {
					System.err.println("Could not load custom home field layout");
					System.err.println(e);
					return kTagLayoutComp;
				}
			} else {
				try {
					return new AprilTagFieldLayout(
							new File(Filesystem.getDeployDirectory(), "fields/april_tag_layouts/2026welded.json")
									.getPath());
				} catch (IOException e) {
					System.err.println("Could not load custom home field layout");
					System.err.println(e);
					return kTagLayoutComp;
				}
			}

		}
	}

	public static final class PathPlannerConstants {
		public static final PathConstraints kPathFollowingConstraints = new PathConstraints(
				0.05, 0.01,
				Units.degreesToRadians(180), Units.degreesToRadians(360));
	}
}