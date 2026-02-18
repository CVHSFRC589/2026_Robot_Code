// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Home extends Command {
  /** Creates a new Home. */
  public ClimberSubsystem m_climberSubsystem;

  public Home(ClimberSubsystem climberSubsystem) {
    m_climberSubsystem = climberSubsystem;
    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_climberSubsystem.m_leftClimber.setSpeed(-1);
    // m_climberSubsystem.m_rightClimber.setSpeed(-1);

    m_climberSubsystem.m_leftClimber.set(-1);
    m_climberSubsystem.m_rightClimber.set(-1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      // m_climberSubsystem.m_leftClimber.setSpeed(0);
      // m_climberSubsystem.m_rightClimber.setSpeed(0);

      m_climberSubsystem.m_leftClimber.set(0);
      m_climberSubsystem.m_rightClimber.set(0);

      m_climberSubsystem.m_leftClimber.resetEncoder();
      m_climberSubsystem.m_rightClimber.resetEncoder();

    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climberSubsystem.m_leftClimber.isLimitSwitchTouched()
        && m_climberSubsystem.m_rightClimber.isLimitSwitchTouched();
  }
}
