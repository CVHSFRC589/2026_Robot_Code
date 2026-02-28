// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelemetrySubsystem extends SubsystemBase {
  private static PowerDistribution m_PDH = new PowerDistribution();

  /** Creates a new TelemetrySubsystem. */
  public TelemetrySubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Robot Voltage", m_PDH.getVoltage());
    SmartDashboard.putNumber("PDH Port 1 Current", m_PDH.getCurrent(1));
    SmartDashboard.putNumber("PDH Port 2 Current", m_PDH.getCurrent(2));
    SmartDashboard.putNumber("PDH Port 3 Current", m_PDH.getCurrent(3));
    SmartDashboard.putNumber("PDH Port 4 Current", m_PDH.getCurrent(4));
    SmartDashboard.putNumber("PDH Port 5 Current", m_PDH.getCurrent(5));
    SmartDashboard.putNumber("PDH Port 6 Current", m_PDH.getCurrent(6));
    SmartDashboard.putNumber("PDH Port 7 Current", m_PDH.getCurrent(7));
    SmartDashboard.putNumber("PDH Port 8 Current", m_PDH.getCurrent(8));
    SmartDashboard.putNumber("PDH Port 9 Current", m_PDH.getCurrent(9));
    SmartDashboard.putNumber("PDH Port 10 Current", m_PDH.getCurrent(10));
    SmartDashboard.putNumber("PDH Port 11 Current", m_PDH.getCurrent(11));
    SmartDashboard.putNumber("PDH Port 12 Current", m_PDH.getCurrent(12));
    SmartDashboard.putNumber("PDH Port 13 Current", m_PDH.getCurrent(13));
    SmartDashboard.putNumber("PDH Port 14 Current", m_PDH.getCurrent(14));
    SmartDashboard.putNumber("PDH Port 15 Current", m_PDH.getCurrent(15));
    SmartDashboard.putNumber("PDH Port 16 Current", m_PDH.getCurrent(16));
    SmartDashboard.putNumber("PDH Port 17 Current", m_PDH.getCurrent(17));
    SmartDashboard.putNumber("PDH Port 18 Current", m_PDH.getCurrent(18));
    SmartDashboard.putNumber("PDH Port 19 Current", m_PDH.getCurrent(19));
    SmartDashboard.putNumber("PDH Port 20 Current", m_PDH.getCurrent(20));
  }
}
