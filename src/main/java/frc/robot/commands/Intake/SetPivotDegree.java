// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetPivotDegree extends Command {
  /** Creates a new SetPivotDegree. */
  private IntakeSubsystem m_intakeSubsystem;
  private double m_angle;
  private ClosedLoopSlot m_slot;

  public SetPivotDegree(IntakeSubsystem intakeSubsystem, double angle, ClosedLoopSlot slot) {
    m_intakeSubsystem = intakeSubsystem;
    m_angle = angle;
    m_slot = slot;
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.setAngle(m_angle, m_slot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.m_pivotController.isAtSetpoint();
  }
}
