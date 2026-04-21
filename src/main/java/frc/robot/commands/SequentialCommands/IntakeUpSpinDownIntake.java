// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SequentialCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Intake.ExtendIntakePID;
import frc.robot.commands.Intake.RetractIntakePID;
import frc.robot.commands.Intake.SetSpeedIntake;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeUpSpinDownIntake extends SequentialCommandGroup {
  IntakeSubsystem m_intakeSubsystem;

  /** Creates a new IntakeUpSpinDownIntake. */
  public IntakeUpSpinDownIntake(IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_intakeSubsystem = intakeSubsystem;
    addCommands(new RetractIntakePID(intakeSubsystem), new InstantCommand(() -> {
      m_intakeSubsystem.m_intakeMotor.set(0);
    }, m_intakeSubsystem));
  }
}
