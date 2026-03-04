// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AgitateBalls extends Command {
  /** Creates a new AgitateBalls. */
  public IntakeSubsystem m_intakeSubsystem;
  private boolean m_direction;
  public AgitateBalls(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_direction = false;
    m_intakeSubsystem.setAngle(80.0,ClosedLoopSlot.kSlot1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(MathUtil.isNear(m_intakeSubsystem.m_pivotController.getSetpoint(), m_intakeSubsystem.getPivotPosition(), 2)){
      if(m_direction){
        m_intakeSubsystem.setAngle(80, ClosedLoopSlot.kSlot1);
        m_direction = false;
      }
      else{
        m_intakeSubsystem.setAngle(40, ClosedLoopSlot.kSlot1);
        m_direction = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
