// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import javax.xml.xpath.XPathVariableResolver;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTag;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller.Button;
// import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.PathfindToPose;
import frc.robot.commands.PointToPose;
import frc.robot.commands.Climber.ExtendClimber;
import frc.robot.commands.Climber.ExtendClimbersDutyCycle;
import frc.robot.commands.Climber.HomeClimber;
import frc.robot.commands.Climber.MoveLeftClimber;
import frc.robot.commands.Climber.MoveRightClimber;
import frc.robot.commands.Climber.RetractClimber;
import frc.robot.commands.Climber.RetractClimbersDutyCycle;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.HomeIntake;
import frc.robot.commands.Intake.RunIntakeOG;
import frc.robot.commands.Intake.SetPivotDegree;
import frc.robot.commands.Intake.SetSpeedIntake;
import frc.robot.commands.Shooter.FeedFuelAtTarget;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.ShootWithVoltage;
import frc.robot.commands.Shooter.SpinToDistanceTargetSpeed;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;
import frc.robot.subsystems.ClimberSubsystem.Climber;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();
	private final PhotonVisionSubsystem m_PhotonVision = new PhotonVisionSubsystem(false,
			m_robotDrive::addVisionMeasurement);
	private final TelemetrySubsystem m_telemetry = new TelemetrySubsystem();
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
	private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
	// public boolean isTest = false;

	// The driver's controller
	XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
	CommandXboxController m_testController = new CommandXboxController(5);

	AprilTagFieldLayout m_fieldLayout;

	Alliance m_alliance;

	private final SendableChooser<Command> autoChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// var aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
		// Configure the button bindings
		m_fieldLayout = FieldConstants.LoadLayout(false);
		System.out.println("April tag layout field width: " + m_fieldLayout.getFieldWidth());
		configureButtonBindings();
		autoChooser = AutoBuilder.buildAutoChooser();
		autoChooser.addOption("Spin Up Shooter", new SpinToDistanceTargetSpeed(m_shooterSubsystem));
		autoChooser.addOption("Extend Intake", new ExtendIntake(m_intakeSubsystem));
		autoChooser.addOption("Run Intake At Max Speed",
				new SetSpeedIntake(m_intakeSubsystem, IntakeConstants.kIntakeFullSpeed));
		SmartDashboard.putData("Auto Chooser", autoChooser);
		m_alliance = DriverStation.getAlliance().get();

		// Configure default commands
		m_robotDrive.setDefaultCommand(
				// The left stick controls translation of the robot.
				// Turning is controlled by the X axis of the right stick.
				new RunCommand(
						() -> m_robotDrive.drive(
								// MathUtil.applyDeadband(m_driverController.getLeftY(),
								// OIConstants.kDriveDeadband),
								Math.pow(MathUtil.applyDeadband(m_driverController.getLeftY(),
										OIConstants.kDriveDeadband), 3),
								// MathUtil.applyDeadband(m_driverController.getLeftX(),
								// OIConstants.kDriveDeadband),
								Math.pow(MathUtil.applyDeadband(m_driverController.getLeftX(),
										OIConstants.kDriveDeadband), 3),
								-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
								true),
						m_robotDrive));
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * 
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */

	private void configureButtonBindings() {
		new JoystickButton(m_driverController, Button.kR1.value)
				.whileTrue(new RunCommand(
						() -> m_robotDrive.setX(),
						m_robotDrive));

		new JoystickButton(m_driverController, XboxController.Button.kY.value)
				.onTrue(new InstantCommand(
						() -> m_robotDrive.zeroHeading(),
						m_robotDrive));

		new JoystickButton(m_driverController, XboxController.Button.kB.value)
				.whileTrue(new PointToPose(
						m_robotDrive,
						() -> getAlliance() ? FieldConstants.kRedHubPose : FieldConstants.kBlueHubPose,
						() -> getDriverControllerProcessedLeftStickY(),
						() -> getDriverControllerProcessedLeftStickX()));

		// new JoystickButton(m_driverController,
		// XboxController.Button.kA.value).onTrue(
		// new PathfindToPose(FieldConstants.kRedTrenchLeftPose));
		new JoystickButton(m_driverController, XboxController.Button.kA.value)
				.whileTrue(new FeedFuelAtTarget(m_shooterSubsystem));
		new JoystickButton(m_testController.getHID(), XboxController.Button.kLeftBumper.value)
				.whileTrue(new SpinToDistanceTargetSpeed(m_shooterSubsystem));
		// m_testController.start().whileTrue(new RunIntake(m_intakeSubsystem));
		m_testController.start().toggleOnTrue(new SetSpeedIntake(m_intakeSubsystem, 0.4));
		m_testController.rightTrigger().whileTrue(new SetSpeedIntake(m_intakeSubsystem, -0.4));
		new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(
				new HomeIntake(m_intakeSubsystem));
		new JoystickButton(m_driverController, XboxController.Button.kX.value)
				.whileTrue(new HomeIntake(m_intakeSubsystem));
		// new JoystickButton(m_driverController,
		// XboxController.Button.kLeftBumper.value)
		// .whileTrue(new HomeClimber(m_climberSubsystem));
		new JoystickButton(m_driverController, XboxController.Button.kBack.value)
				.whileTrue(new ExtendClimber(m_climberSubsystem));
		new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
				.whileTrue(new RetractClimber(m_climberSubsystem));
		new JoystickButton(m_driverController, XboxController.Button.kStart.value)
				.whileTrue(new Shoot(
						() -> {
							return SmartDashboard.getNumber("Top Motor Speed Target", 0);
						},
						() -> {
							return SmartDashboard.getNumber("Middle Motor Speed Target", 0);
						},
						() -> {
							return SmartDashboard.getNumber("Bottom Motor Speed Target", 0);
						},
						m_shooterSubsystem));
		SmartDashboard.putNumber("Top Motor Speed Target", 0);
		SmartDashboard.putNumber("Middle Motor Speed Target", 0);
		SmartDashboard.putNumber("Bottom Motor Speed Target", 0);

		new JoystickButton(m_testController.getHID(), XboxController.Button.kX.value)
				.whileTrue(new MoveLeftClimber(m_climberSubsystem, .1));
		new JoystickButton(m_testController.getHID(), XboxController.Button.kB.value)
				.whileTrue(new MoveLeftClimber(m_climberSubsystem, -.1));
		new JoystickButton(m_testController.getHID(), XboxController.Button.kY.value)
				.whileTrue(new MoveRightClimber(m_climberSubsystem, .1));
		new JoystickButton(m_testController.getHID(), XboxController.Button.kRightBumper.value)
				.whileTrue(new MoveRightClimber(m_climberSubsystem, -.1));
		m_testController.povDown().whileTrue(new RetractClimbersDutyCycle(m_climberSubsystem));
		m_testController.povUp().whileTrue(new ExtendClimbersDutyCycle(m_climberSubsystem));
		m_testController.povLeft().whileTrue(new ExtendIntake(m_intakeSubsystem));
		m_testController.leftStick().whileTrue(new InstantCommand(() -> {
			m_climberSubsystem.m_leftClimber.m_motor.getEncoder().setPosition(0);
			m_climberSubsystem.m_rightClimber.m_motor.getEncoder().setPosition(0);
		}, m_climberSubsystem));
		// auto aim commands (change these to operator board)
		// pass to left setpoint
		new POVButton(m_driverController, 270).whileTrue(
				new PointToPose(
						m_robotDrive,
						() -> getAlliance() ? FieldConstants.kPassingPointRedLeftPose
								: FieldConstants.kPassingPointBlueLeftPose,
						() -> getDriverControllerProcessedLeftStickY(),
						() -> getDriverControllerProcessedLeftStickX()));
		new POVButton(m_driverController, 90).whileTrue(
				new PointToPose(
						m_robotDrive,
						() -> getAlliance() ? FieldConstants.kPassingPointRedRightPose
								: FieldConstants.kPassingPointBlueRightPose,
						() -> getDriverControllerProcessedLeftStickY(),
						() -> getDriverControllerProcessedLeftStickX()));
		new POVButton(m_driverController, 0).whileTrue(
				new SetPivotDegree(m_intakeSubsystem, IntakeConstants.kExtendPivotAngle, ClosedLoopSlot.kSlot0));
		// new JoystickButton(m_driverController, XboxController.Axis.)
		m_testController.a()
				.whileTrue(new InstantCommand(() -> {
					double m_topSpeed = SmartDashboard.getNumber("Top Motor Speed Set", 0);
					double m_middleSpeed = SmartDashboard.getNumber("Middle Motor Speed Set", 0);
					double m_bottomSpeed = SmartDashboard.getNumber("Bottom Motor Speed Set", 0);
					m_shooterSubsystem.setSpeedTop(m_topSpeed);
					m_shooterSubsystem.setSpeedMiddle(m_middleSpeed);
					m_shooterSubsystem.setSpeedBottom(m_bottomSpeed);
					// System.out.println("setting top motor speed to 10");
				}, m_shooterSubsystem))
				.whileFalse(new InstantCommand(() -> {
					m_shooterSubsystem.setSpeedTop(0);
					m_shooterSubsystem.setSpeedMiddle(0);
					m_shooterSubsystem.setSpeedBottom(0);
					// System.out.println("setting top motor speed to 0");
				}, m_shooterSubsystem));

	}

	private double getDriverControllerProcessedLeftStickX() {
		return Math.pow(MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 3);
	}

	private double getDriverControllerProcessedLeftStickY() {
		return Math.pow(MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 3);
	}

	public boolean getAlliance() {
		return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
	}

	static public boolean isHubActive() {
		Optional<Alliance> alliance = DriverStation.getAlliance();
		// If we have no alliance, we cannot be enabled, therefore no hub.
		if (alliance.isEmpty()) {
			return false;
		}
		// Hub is always enabled in autonomous.
		if (DriverStation.isAutonomousEnabled()) {
			return true;
		}
		// At this point, if we're not teleop enabled, there is no hub.
		if (!DriverStation.isTeleopEnabled()) {
			return false;
		}

		// We're teleop enabled, compute.
		double matchTime = DriverStation.getMatchTime();
		String gameData = DriverStation.getGameSpecificMessage();
		// If we have no game data, we cannot compute, assume hub is active, as its
		// likely early in teleop.
		if (gameData.isEmpty()) {
			return true;
		}
		boolean redInactiveFirst = false;
		switch (gameData.charAt(0)) {
			case 'R' -> redInactiveFirst = true;
			case 'B' -> redInactiveFirst = false;
			default -> {
				// If we have invalid game data, assume hub is active.
				return true;
			}
		}

		// Shift was is active for blue if red won auto, or red if blue won auto.
		boolean shift1Active = switch (alliance.get()) {
			case Red -> !redInactiveFirst;
			case Blue -> redInactiveFirst;
		};

		if (matchTime > 130) {
			// Transition shift, hub is active.
			return true;
		} else if (matchTime > 105) {
			// Shift 1
			return shift1Active;
		} else if (matchTime > 80) {
			// Shift 2
			return !shift1Active;
		} else if (matchTime > 55) {
			// Shift 3
			return shift1Active;
		} else if (matchTime > 30) {
			// Shift 4
			return !shift1Active;
		} else {
			// End game, hub always active.
			return true;
		}
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	// public Command getAutonomousCommand() {
	// // Create config for trajectory
	// TrajectoryConfig config = new TrajectoryConfig(
	// AutoConstants.kMaxSpeedMetersPerSecond,
	// AutoConstants.kMaxAccelerationMetersPerSecondSquared)
	// // Add kinematics to ensure max speed is actually obeyed
	// .setKinematics(DriveConstants.kDriveKinematics);

	// // An example trajectory to follow. All units in meters.
	// Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
	// // Start at the origin facing the +X direction
	// new Pose2d(0, 0, new Rotation2d(0)),
	// // Pass through these two interior waypoints, making an 's' curve path
	// List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
	// // End 3 meters straight ahead of where we started, facing forward
	// new Pose2d(3, 0, new Rotation2d(0)),
	// config);

	// var thetaController = new ProfiledPIDController(
	// AutoConstants.kPThetaController, 0, 0,
	// AutoConstants.kThetaControllerConstraints);
	// thetaController.enableContinuousInput(-Math.PI, Math.PI);

	// SwerveControllerCommand swerveControllerCommand = new
	// SwerveControllerCommand(
	// exampleTrajectory,
	// m_robotDrive::getPose, // Functional interface to feed supplier
	// DriveConstants.kDriveKinematics,

	// // Position controllers
	// new PIDController(AutoConstants.kPXController, 0, 0),
	// new PIDController(AutoConstants.kPYController, 0, 0),
	// thetaController,
	// m_robotDrive::setModuleStates,
	// m_robotDrive);

	// // Reset odometry to the starting pose of the trajectory.
	// m_robotDrive.resetPose(exampleTrajectory.getInitialPose());

	// // Run path following command, then stop at the end.
	// return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
	// false));
	// }
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
