// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeClimber extends Command {
	/** Creates a new Home. */
	public ClimberSubsystem m_climberSubsystem;

	public HomeClimber(ClimberSubsystem climberSubsystem) {
		m_climberSubsystem = climberSubsystem;
		addRequirements(m_climberSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_climberSubsystem.m_leftClimber.setSpeed(-ClimberConstants.kHomeDutyCycle);
		m_climberSubsystem.m_rightClimber.setSpeed(-ClimberConstants.kHomeDutyCycle);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (m_climberSubsystem.m_leftClimber.isLimitSwitchTouched()) {
			m_climberSubsystem.m_leftClimber.setSpeed(0);
			m_climberSubsystem.m_leftClimber.resetEncoder();
		}

		if (m_climberSubsystem.m_rightClimber.isLimitSwitchTouched()) {
			m_climberSubsystem.m_rightClimber.setSpeed(0);
			m_climberSubsystem.m_leftClimber.resetEncoder();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_climberSubsystem.m_leftClimber.isLimitSwitchTouched()
				&& m_climberSubsystem.m_rightClimber.isLimitSwitchTouched();
	}
}
