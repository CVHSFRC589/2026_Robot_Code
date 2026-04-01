// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Utils;

public class TelemetrySubsystem extends SubsystemBase {
  private static PowerDistribution m_PDH = new PowerDistribution();
  // private CommandXboxController m_driverController, m_operatorController;
  private static List<CommandXboxController> m_controllerList = new ArrayList<CommandXboxController>();

  /** Creates a new TelemetrySubsystem. */
  public TelemetrySubsystem(CommandXboxController... controllers) {
    for (var controller : controllers) {
      m_controllerList.add(controller);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putNumber("Robot Voltage", m_PDH.getVoltage());
    // SmartDashboard.putNumber("PDH Port 1 Current", m_PDH.getCurrent(1));
    // SmartDashboard.putNumber("PDH Port 2 Current", m_PDH.getCurrent(2));
    // SmartDashboard.putNumber("PDH Port 3 Current", m_PDH.getCurrent(3));
    // SmartDashboard.putNumber("PDH Port 4 Current", m_PDH.getCurrent(4));
    // SmartDashboard.putNumber("PDH Port 5 Current", m_PDH.getCurrent(5));
    // SmartDashboard.putNumber("PDH Port 6 Current", m_PDH.getCurrent(6));
    // SmartDashboard.putNumber("PDH Port 7 Current", m_PDH.getCurrent(7));
    // SmartDashboard.putNumber("PDH Port 8 Current", m_PDH.getCurrent(8));
    // SmartDashboard.putNumber("PDH Port 9 Current", m_PDH.getCurrent(9));
    // SmartDashboard.putNumber("PDH Port 10 Current", m_PDH.getCurrent(10));
    // SmartDashboard.putNumber("PDH Port 11 Current", m_PDH.getCurrent(11));
    // SmartDashboard.putNumber("PDH Port 12 Current", m_PDH.getCurrent(12));
    // SmartDashboard.putNumber("PDH Port 13 Current", m_PDH.getCurrent(13));
    // SmartDashboard.putNumber("PDH Port 14 Current", m_PDH.getCurrent(14));
    // SmartDashboard.putNumber("PDH Port 15 Current", m_PDH.getCurrent(15));
    // SmartDashboard.putNumber("PDH Port 16 Current", m_PDH.getCurrent(16));
    // SmartDashboard.putNumber("PDH Port 17 Current", m_PDH.getCurrent(17));
    // SmartDashboard.putNumber("PDH Port 18 Current", m_PDH.getCurrent(18));
    // SmartDashboard.putNumber("PDH Port 19 Current", m_PDH.getCurrent(19));
    // SmartDashboard.putNumber("PDH Port 20 Current", m_PDH.getCurrent(20));
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putBoolean("Active Hub", isBlueHubActive());
    setControllerRumble();
  }

  static public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  static public boolean isBlueHubActive() {
    Alliance alliance = Alliance.Blue;
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  public static void setControllerRumble() {
    double matchTime = DriverStation.getMatchTime();

    if (matchTime < 110 && matchTime > 105) {
      System.out.println("rumbling controller");
      System.out.println(Utils.Lerp(0, 1, (110 - matchTime) / 5));
      setRumbleAllControllers(Utils.Lerp(0, 1, (110 - matchTime) / 5));
    } else if (matchTime < 85 && matchTime > 80) {
      System.out.println("rumbling controller");
      setRumbleAllControllers(Utils.Lerp(0, 1, (85 - matchTime) / 5));
    } else if (matchTime < 60 && matchTime > 55) {
      System.out.println("rumbling controller");
      setRumbleAllControllers(Utils.Lerp(0, 1, (60 - matchTime) / 5));
    } else if (matchTime < 35 && matchTime > 30) {
      System.out.println("rumbling controller");
      setRumbleAllControllers(Utils.Lerp(0, 1, (35 - matchTime) / 5));
    } else {
      setRumbleAllControllers(0);
    }
  }

  private static void setRumbleAllControllers(double value) {
    for (var controller : m_controllerList) {
      controller.setRumble(RumbleType.kBothRumble, value);
    }
  }
}