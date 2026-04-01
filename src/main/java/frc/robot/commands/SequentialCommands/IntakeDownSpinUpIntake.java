// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SequentialCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Intake.ExtendIntakePID;
import frc.robot.commands.Intake.SetSpeedIntake;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeDownSpinUpIntake extends SequentialCommandGroup {
  IntakeSubsystem m_intakeSubsystem;
  /** Creates a new IntakeDownSpinUpIntake. */
  public IntakeDownSpinUpIntake(IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_intakeSubsystem = intakeSubsystem;
    addCommands(new ExtendIntakePID(intakeSubsystem), new SetSpeedIntake(m_intakeSubsystem, IntakeConstants.kIntakeFullSpeed));
  }
}
