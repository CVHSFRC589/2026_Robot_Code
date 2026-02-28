// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.ClosedLoopSlot;

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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.PathfindToPose;
import frc.robot.commands.PointToPose;
import frc.robot.commands.Shoot;
import frc.robot.commands.Climber.ExtendClimber;
import frc.robot.commands.Climber.HomeClimber;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.HomeIntake;
import frc.robot.commands.Intake.SetPivotDegree;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;

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

		new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(
				new PathfindToPose(FieldConstants.kRedTrenchLeftPose));

		new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(
				new HomeIntake(m_intakeSubsystem));
		new JoystickButton(m_driverController, XboxController.Button.kX.value)
				.whileTrue(new ExtendIntake(m_intakeSubsystem));
		new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
				.whileTrue(new HomeClimber(m_climberSubsystem));
		new JoystickButton(m_driverController, XboxController.Button.kBack.value)
				.whileTrue(new ExtendClimber(m_climberSubsystem));
		new JoystickButton(m_driverController, XboxController.Button.kStart.value)
				.whileTrue(new Shoot(1500.0, 3000.0, 1000.0, m_shooterSubsystem));
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
				new SetPivotDegree(m_intakeSubsystem, 160, ClosedLoopSlot.kSlot0));
		// new JoystickButton(m_driverController, XboxController.Axis.)
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
