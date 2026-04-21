// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import javax.xml.xpath.XPathVariableResolver;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTag;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS4Controller.Button;
// import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.PathfindToPose;
import frc.robot.commands.PointToPose;
import frc.robot.commands.SequentialCommands.IntakeDownSpinUpIntake;
import frc.robot.commands.Climber.ExtendClimber;
import frc.robot.commands.Climber.ExtendClimbersDutyCycle;
import frc.robot.commands.Climber.HomeClimber;
import frc.robot.commands.Climber.MoveLeftClimber;
import frc.robot.commands.Climber.MoveRightClimber;
import frc.robot.commands.Climber.RetractClimber;
import frc.robot.commands.Climber.RetractClimbersDutyCycle;
import frc.robot.commands.Intake.AgitateBalls;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.ExtendIntakePID;
import frc.robot.commands.Intake.HomeIntake;
import frc.robot.commands.Intake.RetractIntakePID;
import frc.robot.commands.Intake.RunIntakeOG;
import frc.robot.commands.Intake.SetPivotDegree;
import frc.robot.commands.Intake.SetSpeedIntake;
import frc.robot.commands.Intake.SpinDownIntake;
import frc.robot.commands.Intake.SpinUpDownIntake;
import frc.robot.commands.Intake.SpinUpIntake;
import frc.robot.commands.SequentialCommands.IntakeUpSpinDownIntake;
import frc.robot.commands.Shooter.FeedFuelAtTarget;
import frc.robot.commands.Shooter.FeedPass;
import frc.robot.commands.Shooter.FeedShooter;
import frc.robot.commands.Shooter.ReverseKickupMotor;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.ShootWithVoltage;
import frc.robot.commands.Shooter.SpinDown;
import frc.robot.commands.Shooter.SpinToDistanceTargetSpeed;
import frc.robot.commands.Shooter.SpinUpPass;
import frc.robot.commands.Shooter.SpinUpShooter;
import frc.robot.commands.Shooter.SpinUpToInputTargets;
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
	CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
	CommandXboxController m_operatorController = new CommandXboxController(1);
	CommandXboxController m_testController = new CommandXboxController(5);

	// The robot's subsystems
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();
	private final PhotonVisionSubsystem m_PhotonVision = new PhotonVisionSubsystem(false,
			m_robotDrive::addVisionMeasurement);
	private final TelemetrySubsystem m_telemetry = new TelemetrySubsystem(m_driverController, m_operatorController);
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
	private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
	private final PathPlannerLogging m_pathPlannerLogging = new PathPlannerLogging();
	// public boolean isTest = false;

	// The driver's controller

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

		// configure controllers
		// configureButtonBindings();
		configureDriverControllerButtonBindings();
		configureOperatorControllerButtonBindings();
		configureTestControllerButtonBindings();

		autoChooser = AutoBuilder.buildAutoChooser();
		// autoChooser.addOption("Spin Up Shooter", new
		// SpinToDistanceTargetSpeed(m_shooterSubsystem));
		// autoChooser.addOption("Extend Intake", new ExtendIntake(m_intakeSubsystem));
		// autoChooser.addOption("Run Intake At Max Speed",
		// new SetSpeedIntake(m_intakeSubsystem, IntakeConstants.kIntakeFullSpeed));

		// shooter commands
		NamedCommands.registerCommand("Spin Up Shooter", new SpinUpShooter(m_shooterSubsystem));
		NamedCommands.registerCommand("Spin Down Shooter", new SpinDown(m_shooterSubsystem));
		NamedCommands.registerCommand("Shoot", new Shoot(m_shooterSubsystem));
		NamedCommands.registerCommand("Feed Shooter", new FeedShooter(m_shooterSubsystem));

		// climber commands
		NamedCommands.registerCommand("Home Climber", new HomeClimber(m_climberSubsystem));
		NamedCommands.registerCommand("Raise Climber", new ExtendClimber(m_climberSubsystem));
		NamedCommands.registerCommand("Lower Climber", new RetractClimber(m_climberSubsystem));

		// intake commands
		NamedCommands.registerCommand("Lower Intake", new ExtendIntakePID(m_intakeSubsystem));
		NamedCommands.registerCommand("Raise Intake", new RetractIntakePID(m_intakeSubsystem));
		NamedCommands.registerCommand("Spin Up Intake",
				new SetSpeedIntake(m_intakeSubsystem, IntakeConstants.kIntakeFullSpeed));
		NamedCommands.registerCommand("Spin Down Intake", new InstantCommand(() -> {
			m_intakeSubsystem.m_intakeMotor.set(0);
		}, m_intakeSubsystem));

		// intake
		new EventTrigger("Lower Intake").onTrue(new ExtendIntakePID(m_intakeSubsystem));
		new EventTrigger("Raise Intake").onTrue(new RetractIntakePID(m_intakeSubsystem));
		new EventTrigger("Spin Up Intake").onTrue(new SpinUpIntake(m_intakeSubsystem));
		new EventTrigger("Spin Down Intake").onTrue(new SpinDownIntake(m_intakeSubsystem));
		new EventTrigger("Spin Up Down Intake").whileTrue(new SpinUpDownIntake(m_intakeSubsystem));

		// climber
		new EventTrigger("Raise Climber").onTrue(new ExtendClimber(m_climberSubsystem));
		new EventTrigger("Lower Climber").onTrue(new RetractClimber(m_climberSubsystem));

		autoChooser.addOption("Left Start - Shoot Preload - Go to Left Outside Climb - NO CLIMB",
				new PathPlannerAuto("Left Start - Shoot Preload - Go to Left Outside Climb - NO CLIMB"));
		autoChooser.addOption("Left Trench Start - Wait - Shoot Preload - Left Outside Climb",
				new PathPlannerAuto("Left Trench Start - Wait - Shoot Preload - Left Outside Climb"));
		autoChooser.addOption("Middle Start - Left Outside Climb",
				new PathPlannerAuto("Middle Start - Left Outside Climb"));
		autoChooser.addOption("Left Start - Preload Shoot - Out of Way",
				new PathPlannerAuto("Left Shoot - Preload Shoot - Out of Way"));
		autoChooser.addOption("Left Start - Trench - Neutral Pass - Trench - Shoot - Left Outside Climb",
				new PathPlannerAuto("Left Start - Trench - Neutral Pass - Trench - Shoot - Left Outside Climb"));
		autoChooser.addOption("Right Bump - Right Outside Climb",
				new PathPlannerAuto("Right Bump - Right Outside Climb"));
		autoChooser.addOption("Center Start - Straight Back - Left Outside Climb",
				new PathPlannerAuto("Center Start - Straight Back - Left Outside Climb"));
		autoChooser.addOption("Center Start - Straight Back - Right Outside Climb",
				new PathPlannerAuto("Center Start - Straight Back - Right Outside Climb"));

		// autoChooser.addOption("Right Start - Preload Shoot - Right Outside
		// Climb");
		SmartDashboard.putData("Auto Chooser", autoChooser);
		m_alliance = DriverStation.getAlliance().get();

		// Configure default commands
		m_robotDrive.setDefaultCommand(
				// The left stick controls translation of the robot.
				// Turning is controlled by the X axis of the right stick.
				new RunCommand(
						() -> m_robotDrive.drive(
								// MathUtil.applyDeadband(m_driverController.getHID().getLeftY(),
								// OIConstants.kDriveDeadband),
								getDriverControllerProcessedLeftStickY(),
								// MathUtil.applyDeadband(m_driverController.getHID().getLeftX(),
								// OIConstants.kDriveDeadband),
								getDriverControllerProcessedLeftStickX(),
								-getDriverControllerProcessedRightStickX(),
								true),
						m_robotDrive));
		m_shooterSubsystem.setDefaultCommand(
				new InstantCommand(() -> {
					// m_shooterSubsystem.setSpeedBottom(-150);
					m_shooterSubsystem.moveBottomMotor(-0.2);
				}, m_shooterSubsystem));
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
		new JoystickButton(m_driverController.getHID(), Button.kR1.value)
				.whileTrue(new RunCommand(
						() -> m_robotDrive.setX(),
						m_robotDrive));

		new JoystickButton(m_driverController.getHID(), XboxController.Button.kY.value)
				.onTrue(new InstantCommand(
						() -> m_robotDrive.zeroHeading(),
						m_robotDrive));

		new JoystickButton(m_driverController.getHID(), XboxController.Button.kB.value)
				.whileTrue(new PointToPose(
						m_robotDrive,
						() -> isRedAlliance() ? FieldConstants.kRedHubPose : FieldConstants.kBlueHubPose,
						() -> getDriverControllerProcessedLeftStickY(),
						() -> getDriverControllerProcessedLeftStickX()));

		// new JoystickButton(m_driverController,
		// XboxController.Button.kA.value).onTrue(
		// new PathfindToPose(FieldConstants.kRedTrenchLeftPose));
		new JoystickButton(m_driverController.getHID(), XboxController.Button.kA.value)
				.whileTrue(new FeedFuelAtTarget(m_shooterSubsystem));
		new JoystickButton(m_testController.getHID(), XboxController.Button.kLeftBumper.value)
				.whileTrue(new SpinToDistanceTargetSpeed(m_shooterSubsystem));
		// m_testController.start().whileTrue(new RunIntake(m_intakeSubsystem));
		m_testController.start().whileTrue(new SetSpeedIntake(m_intakeSubsystem, 3000));
		m_testController.rightTrigger().whileTrue(new SetSpeedIntake(m_intakeSubsystem, -3000));
		new JoystickButton(m_driverController.getHID(), XboxController.Button.kRightBumper.value).whileTrue(
				new HomeIntake(m_intakeSubsystem));
		new JoystickButton(m_driverController.getHID(), XboxController.Button.kX.value)
				.whileTrue(new HomeIntake(m_intakeSubsystem));
		// new JoystickButton(m_driverController,
		// XboxController.Button.kLeftBumper.value)
		// .whileTrue(new HomeClimber(m_climberSubsystem));
		new JoystickButton(m_driverController.getHID(), XboxController.Button.kBack.value)
				.whileTrue(new ExtendClimber(m_climberSubsystem));
		new JoystickButton(m_driverController.getHID(), XboxController.Button.kLeftBumper.value)
				.whileTrue(new RetractClimber(m_climberSubsystem));
		new JoystickButton(m_driverController.getHID(), XboxController.Button.kStart.value)
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
		new POVButton(m_driverController.getHID(), 270).whileTrue(
				new PointToPose(
						m_robotDrive,
						() -> isRedAlliance() ? FieldConstants.kPassingPointRedLeftPose
								: FieldConstants.kPassingPointBlueLeftPose,
						() -> getDriverControllerProcessedLeftStickY(),
						() -> getDriverControllerProcessedLeftStickX()));
		new POVButton(m_driverController.getHID(), 90).whileTrue(
				new PointToPose(
						m_robotDrive,
						() -> isRedAlliance() ? FieldConstants.kPassingPointRedRightPose
								: FieldConstants.kPassingPointBlueRightPose,
						() -> getDriverControllerProcessedLeftStickY(),
						() -> getDriverControllerProcessedLeftStickX()));
		new POVButton(m_driverController.getHID(), 0).whileTrue(
				new SetPivotDegree(m_intakeSubsystem, IntakeConstants.kIntakeExtendedPosition, ClosedLoopSlot.kSlot0));
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

	private void configureDriverControllerButtonBindings() {
		new JoystickButton(m_testController.getHID(), XboxController.Button.kX.value)
				.whileTrue(new MoveLeftClimber(m_climberSubsystem, .1));
		new JoystickButton(m_testController.getHID(), XboxController.Button.kB.value)
				.whileTrue(new MoveLeftClimber(m_climberSubsystem, -.1));
		new JoystickButton(m_testController.getHID(), XboxController.Button.kY.value)
				.whileTrue(new MoveRightClimber(m_climberSubsystem, .1));
		new JoystickButton(m_testController.getHID(), XboxController.Button.kRightBumper.value)
				.whileTrue(new MoveRightClimber(m_climberSubsystem, -.1));

		SmartDashboard.putNumber("Top Motor Speed Target", 0);
		SmartDashboard.putNumber("Middle Motor Speed Target", 0);
		SmartDashboard.putNumber("Bottom Motor Speed Target", 0);
		m_driverController.rightTrigger()
				// .whileTrue(new Shoot(
				// () -> {
				// return SmartDashboard.getNumber("Top Motor Speed Target", 0);
				// },

				// () -> {
				// return SmartDashboard.getNumber("Middle Motor Speed Target", 0);
				// },
				// () -> {
				// return SmartDashboard.getNumber("Bottom Motor Speed Target", 0);
				// },
				// m_shooterSubsystem));
				.whileTrue(new FeedShooter(m_shooterSubsystem));
		m_driverController.leftTrigger()
				.whileTrue(new PointToPose(m_robotDrive,
						() -> {
							return isRedAlliance() ? FieldConstants.kRedHubPose : FieldConstants.kBlueHubPose;
						},
						() -> {
							return getDriverControllerProcessedLeftStickY() / 2;
						},
						() -> {
							return getDriverControllerProcessedLeftStickX() / 2;
						}));
		m_driverController.leftBumper()
				.whileTrue(new ExtendClimber(m_climberSubsystem));
		m_driverController.rightBumper()
				.whileTrue(new RetractClimber(m_climberSubsystem));
		m_driverController.y().onTrue(new InstantCommand(
				() -> m_robotDrive.zeroHeading(),
				m_robotDrive));
		m_driverController.b().whileTrue(new FeedPass(m_shooterSubsystem));
		// .whileTrue(new Shoot(
		// () -> {
		// return 4000; // 4000
		// },
		// () -> {
		// return 6500; // 6500
		// },
		// () -> {

		// return 1500;
		// },
		// m_shooterSubsystem));
	}

	private void configureOperatorControllerButtonBindings() {
		// shooter
		// m_operatorController.leftTrigger().whileTrue(new
		// SpinUpToInputTargets(m_shooterSubsystem));
		m_operatorController.rightTrigger()
				.whileTrue(new SpinUpPass(m_shooterSubsystem))
				.whileFalse(new SpinDown(m_shooterSubsystem));
		// .whileTrue(new Shoot(
		// () -> {
		// return 4000; // 4000
		// },
		// () -> {
		// return 5500; // 6500
		// },
		// () -> {
		// return -250;
		// },
		// m_shooterSubsystem));
		// m_operatorController.leftTrigger().whileTrue(new Shoot(
		// () -> {
		// // return SmartDashboard.getNumber("Top Motor Speed Target", 0);
		// return 3750;
		// },
		// () -> {

		// // return SmartDashboard.getNumber("Middle Motor Speed Target", 0);
		// return 1500;
		// },
		// () -> {
		// return 0;
		// },
		// m_shooterSubsystem));
		m_operatorController.leftTrigger().whileTrue(new SpinUpShooter(m_shooterSubsystem))
				.whileFalse(new SpinDown(m_shooterSubsystem));

		// m_operatorController.a().toggleOnTrue(new
		// SpinUpToInputTargets(m_shooterSubsystem));
		m_operatorController.a().whileTrue(new ReverseKickupMotor(m_shooterSubsystem));
		// m_operatorController.b().whileTrue(new SpinUpToPassSpeeds());
		// intake
		// m_operatorController.leftBumper().whileTrue(new
		// ExtendIntake(m_intakeSubsystem));
		// // m_operatorController.rightBumper().whileTrue(new RetractIntake());
		// m_operatorController.rightBumper().whileTrue(new
		// HomeIntake(m_intakeSubsystem));
		m_operatorController.povLeft()
				.onTrue(new SpinUpIntake(m_intakeSubsystem))
				.onFalse(new SpinDownIntake(m_intakeSubsystem));
		m_operatorController.povRight()
				.onTrue(new SpinUpIntake(m_intakeSubsystem))
				.onFalse(new SpinDownIntake(m_intakeSubsystem));

		// sequences
		// m_operatorController.povDown().whileTrue(
		// new SequentialCommandGroup(new ExtendIntakePID(m_intakeSubsystem),
		// new SetSpeedIntake(m_intakeSubsystem, IntakeConstants.kIntakeFullSpeed)));
		// m_operatorController.povUp().whileTrue(
		// new SequentialCommandGroup(new RetractIntakePID(m_intakeSubsystem),
		// new SetSpeedIntake(m_intakeSubsystem, 0)));
		m_operatorController.b().whileTrue(new FeedShooter(m_shooterSubsystem));
		m_operatorController.leftStick().onTrue(new HomeClimber(m_climberSubsystem));
		// m_operatorController.rightStick().onTrue(new InstantCommand(() -> {
		// m_climberSubsystem.m_leftClimber.resetEncoder();
		// m_climberSubsystem.m_rightClimber.resetEncoder();
		// }, m_climberSubsystem));

		m_operatorController.leftBumper().onTrue(new ExtendIntakePID(m_intakeSubsystem));
		m_operatorController.rightBumper().onTrue(new RetractIntakePID(m_intakeSubsystem));
		// m_operatorController.leftBumper().onTrue(new
		// IntakeDownSpinUpIntake(m_intakeSubsystem));
		// m_operatorController.rightBumper().onTrue(new
		// IntakeUpSpinDownIntake(m_intakeSubsystem));
		m_operatorController.x().toggleOnTrue(new AgitateBalls(m_intakeSubsystem));

		m_operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.9)
				.onTrue(
						new InstantCommand(
								() -> {
									m_shooterSubsystem.m_topMotorSpeedOffset += 25;
								}, m_shooterSubsystem));
		m_operatorController.axisLessThan(XboxController.Axis.kLeftY.value, 0.9)
				.onTrue(
						new InstantCommand(
								() -> {
									m_shooterSubsystem.m_topMotorSpeedOffset -= 25;
								}, m_shooterSubsystem));
		m_operatorController.axisGreaterThan(XboxController.Axis.kRightY.value, 0.9)
				.onTrue(
						new InstantCommand(
								() -> {
									m_shooterSubsystem.m_middleMotorSpeedOffset += 25;
								}, m_shooterSubsystem));
		m_operatorController.axisLessThan(XboxController.Axis.kRightY.value, 0.9)
				.onTrue(
						new InstantCommand(
								() -> {
									m_shooterSubsystem.m_middleMotorSpeedOffset -= 25;
								}, m_shooterSubsystem));
		m_operatorController.y()
				.onTrue(
						new InstantCommand(
								() -> {

									m_shooterSubsystem.m_bottomMotorSpeedOffset += 25;
								}, m_shooterSubsystem));
		m_operatorController.x()
				.onTrue(
						new InstantCommand(
								() -> {
									m_shooterSubsystem.m_bottomMotorSpeedOffset -= 25;
								}, m_shooterSubsystem));

	}

	private void configureTestControllerButtonBindings() {
		// y -> top motor speed += 25
		// x -> top motor speed -= 25
		// b -> middle motor speed += 25
		// a -> middle motor speed -= 25
		// start -> bottom motor speed += 25
		// back -> bottom motor speed -= 25
		m_testController.a().onTrue(new InstantCommand(() -> {
			m_shooterSubsystem.setSpeedMiddle(m_shooterSubsystem.getMiddleMotorSpeedTarget() - 100);
		}, m_shooterSubsystem));
		m_testController.b().onTrue(new InstantCommand(() -> {
			m_shooterSubsystem.setSpeedMiddle(m_shooterSubsystem.getMiddleMotorSpeedTarget() + 100);
		}, m_shooterSubsystem));
		m_testController.y().onTrue(new InstantCommand(() -> {
			m_shooterSubsystem.setSpeedTop(m_shooterSubsystem.getTopMotorSpeedTarget() + 100);
		}, m_shooterSubsystem));
		m_testController.x().onTrue(new InstantCommand(() -> {
			m_shooterSubsystem.setSpeedTop(m_shooterSubsystem.getTopMotorSpeedTarget() - 100);
		}, m_shooterSubsystem));
		m_testController.start().onTrue(new InstantCommand(() -> {
			m_shooterSubsystem.setSpeedBottom(m_shooterSubsystem.getBottomMotorSpeedTarget() + 100);
		}, m_shooterSubsystem));
		m_testController.back().onTrue(new InstantCommand(() -> {
			m_shooterSubsystem.setSpeedBottom(m_shooterSubsystem.getBottomMotorSpeedTarget() - 100);
		}, m_shooterSubsystem));
		m_testController.rightBumper()
				.onTrue(new PathPlannerAuto("Center Start - Straight Back - Right Outside Climb"));
	}

	private double getDriverControllerProcessedLeftStickX() {
		return Math.pow(MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 1);
	}

	private double getDriverControllerProcessedLeftStickY() {
		return Math.pow(MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 1);
	}

	private double getDriverControllerProcessedRightStickX() {
		return Math.pow(MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband), 1);
	}

	public boolean isRedAlliance() {
		return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
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
