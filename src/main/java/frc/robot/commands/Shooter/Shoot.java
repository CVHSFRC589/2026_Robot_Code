// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  DoubleSupplier m_topSpeed, m_middleSpeed, m_bottomSpeed;
  ShooterSubsystem m_shooterSubsystem;

  /** Creates a new Shoot. */
  // public Shoot(double topSpeed, double middleSpeed, double bottomSpeed,
  // ShooterSubsystem shooterSubsystem) {
  // // Use addRequirements() here to declare subsystem dependencies.
  // m_topSpeed = topSpeed;
  // m_middleSpeed = middleSpeed;
  // m_bottomSpeed = bottomSpeed;
  // m_shooterSubsystem = shooterSubsystem;
  // addRequirements(m_shooterSubsystem);
  // }

  public Shoot(DoubleSupplier topSpeed, DoubleSupplier middleSpeed, DoubleSupplier bottomSpeed,
      ShooterSubsystem shooterSubsystem) {
    m_topSpeed = topSpeed;
    m_middleSpeed = middleSpeed;
    m_bottomSpeed = bottomSpeed;
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_shooterSubsystem);
  }

  public Shoot(ShooterSubsystem shooterSubsystem) {
    m_topSpeed = () -> {
      return ShooterConstants.kBottomMotorSpinUpSpeed;
    };
    m_middleSpeed = () -> {
      return ShooterConstants.kMiddleMotorSpinUpSpeed;

    };
    m_bottomSpeed = () -> {
      return ShooterConstants.kBottomMotorSpinUpSpeed;
    };
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setSpeedTop(m_topSpeed.getAsDouble());
    System.out.println("Top Speed: " + m_topSpeed.getAsDouble());
    m_shooterSubsystem.setSpeedMiddle(m_middleSpeed.getAsDouble());
    System.out.println("Middle Speed: " + m_middleSpeed.getAsDouble());
    m_shooterSubsystem.setSpeedBottom(m_bottomSpeed.getAsDouble());
    System.out.println("Bottom Speed: " + m_bottomSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop motors here
    m_shooterSubsystem.setSpeedTop(0);
    m_shooterSubsystem.setSpeedMiddle(0);
    m_shooterSubsystem.setSpeedBottom(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
