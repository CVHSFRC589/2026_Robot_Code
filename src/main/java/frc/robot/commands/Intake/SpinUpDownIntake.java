// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinUpDownIntake extends Command {
  IntakeSubsystem m_intakeSubsystem;

  /** Creates a new SpinUpDownIntake. */
  public SpinUpDownIntake(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(m_intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.m_intakeController.setSetpoint(IntakeConstants.kIntakeFullSpeed, ControlType.kVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.m_intakeMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
