// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.RobotManager;
import frc.robot.commands.RobotMode.CycleMode;
import frc.robot.commands.RobotState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climberwheel.ClimberWheelSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainState;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.vision.LimelightLocalization;

public class LED extends SubsystemBase {
  private final AddressableLED glowjack_horseman;
  private final  RobotManager robotManager;
  private final AddressableLEDBuffer m_ledBuffer;
  private final Timer blinkTimer = new Timer();
  private LEDState state = new LEDState(Color.kBlue);

  public LED(RobotManager robotManager) {
    this.robotManager = robotManager;
    blinkTimer.start();
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    glowjack_horseman = new AddressableLED(0);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(150);
    glowjack_horseman.setLength(m_ledBuffer.getLength());
    // Set the data
    glowjack_horseman.setData(m_ledBuffer);
    glowjack_horseman.start();
  }
  @Override
  public void periodic() {
    if (RobotManager.getInstance().getState() == RobotState.PREPARE_HOMING || RobotManager.getInstance().getState() == RobotState.HOMING_STAGE_1_ELEVATOR || RobotManager.getInstance().getState() == RobotState.HOMING_STAGE_2_ELBOW || RobotManager.getInstance().getState() == RobotState.HOMING_STAGE_3_WRIST) {
      LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
    }
    
    if (DrivetrainSubsystem.getInstance().getState() == DrivetrainState.BARGE_ALIGN){
      switch (LimelightLocalization.getInstance().getCoralStationAlignmentState(false)) {
        case ALIGNED:
          LEDPattern.solid(Color.kGreen).applyTo(m_ledBuffer);
          break;
        case NOT_ALIGNED:
          LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
          break;
        case NOT_ALIGNED_FORWARD:
          LEDPattern.solid(Color.kOrange).applyTo(m_ledBuffer);
          break;
        case INVALID:
          LEDPattern.solid(Color.kYellow).applyTo(m_ledBuffer);
          break;
        }
      }

    if (DrivetrainSubsystem.getInstance().getState() == DrivetrainState.TELEOP_CORAL_STATION_ALIGN){
      switch (LimelightLocalization.getInstance().getCoralStationAlignmentState(false)) {
      case ALIGNED:
        LEDPattern.solid(Color.kGreen).applyTo(m_ledBuffer);
        break;
      case NOT_ALIGNED:
        LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
        break;
      case NOT_ALIGNED_FORWARD:
        LEDPattern.solid(Color.kOrange).applyTo(m_ledBuffer);
        break;  
      case INVALID:
        LEDPattern.solid(Color.kYellow).applyTo(m_ledBuffer);
        break;
      }
        }
        else if (DrivetrainSubsystem.getInstance().getState() == DrivetrainState.AUTO_CORAL_STATION_ALIGN_1){
          switch (LimelightLocalization.getInstance().getCoralStationAlignmentState(true)) {
          case ALIGNED:
            LEDPattern.solid(Color.kGreen).applyTo(m_ledBuffer);
            break;
          case NOT_ALIGNED:
            LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
            break;
          case NOT_ALIGNED_FORWARD:
            LEDPattern.solid(Color.kOrange).applyTo(m_ledBuffer);
            break;
          case INVALID:
            LEDPattern.solid(Color.kYellow).applyTo(m_ledBuffer);
            break;
          }
        }
    else if (!RobotManager.getInstance().isHeightCapped || (RobotManager.getInstance().getState() == RobotState.WAIT_L2) || (RobotManager.getInstance().getState() == RobotState.WAIT_L3)){
      switch (LimelightLocalization.getInstance().getReefAlignmentState()) {
      case ALIGNED:
        LEDPattern.solid(Color.kGreen).applyTo(m_ledBuffer);
        break;
      case NOT_ALIGNED:
        LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
        break;
      case NOT_ALIGNED_FORWARD:
        LEDPattern.solid(Color.kOrange).applyTo(m_ledBuffer);
        break;
      case INVALID:
        LEDPattern.solid(Color.kYellow).applyTo(m_ledBuffer);
        break;
      }
    }
      else if (DrivetrainSubsystem.getInstance().getState() == DrivetrainState.AUTO_REEF_ALIGN_1 || DrivetrainSubsystem.getInstance().getState() == DrivetrainState.AUTO_REEF_ALIGN_2){
        switch (LimelightLocalization.getInstance().getReefAlignmentState()) {
        case ALIGNED:
          LEDPattern.solid(Color.kGreen).applyTo(m_ledBuffer);
          break;
        case NOT_ALIGNED:
          LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
          break;
        case NOT_ALIGNED_FORWARD:
          LEDPattern.solid(Color.kOrange).applyTo(m_ledBuffer);
          break;
        case INVALID:
          LEDPattern.solid(Color.kYellow).applyTo(m_ledBuffer);
          break;
        }
    }

    else{
      if (robotManager.getState() == RobotState.DEEP_CLIMB_WAIT || RobotState.DEEP_CLIMB_DEPLOY == robotManager.getState() || RobotState.DEEP_CLIMB_UNLATCH == robotManager.getState() || RobotState.DEEP_CLIMB_RETRACT == robotManager.getState() || RobotState.DEEP_CLIMB_UNWIND == robotManager.getState()) {
        switch (ClimberSubsystem.getInstance().getState()) {
          case DEEP_CLIMB_UNLATCH:
            LEDPattern.solid(Color.kYellow).applyTo(m_ledBuffer);
            break;
          case DEEP_CLIMB_DEPLOY:
            LEDPattern.solid(Color.kYellow).applyTo(m_ledBuffer);
            break;
          case DEEP_CLIMB_WAIT:
            if (ClimberWheelSubsystem.getInstance().hasCage()) {
              LEDPattern.solid(Color.kGreen).applyTo(m_ledBuffer);
            } else {
              LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
            }
            break;
          case DEEP_CLIMB_RETRACT:
            LEDPattern.solid(Color.kGreen).applyTo(m_ledBuffer);
            break;
          case DEEP_CLIMB_UNWIND:
            if (ClimberWheelSubsystem.getInstance().hasCage()) {
              LEDPattern.solid(Color.kGreen).applyTo(m_ledBuffer);
            } else {
              LEDPattern.solid(Color.kRed).applyTo(m_ledBuffer);
            }
            break;
          default:
            break;
        }
      } else {
        switch (RobotManager.getInstance().currentGameMode) {
          case CORAL:
              if (RobotManager.getInstance().currentCycleMode == CycleMode.SUPERCYCLE) {
                LEDPattern.solid(Color.kOrangeRed).applyTo(m_ledBuffer);
              } else {
                LEDPattern.solid(Color.kCoral).applyTo(m_ledBuffer);
              }
            break;
          case ALGAE:
             LEDPattern.solid(Color.kBlue).applyTo(m_ledBuffer);
            break;
          default:
            break;
        }
      }
    }
    if (DriverStation.isDisabled()) {
      LEDPattern.solid(Color.kPurple).applyTo(m_ledBuffer);
    }

    glowjack_horseman.setData(m_ledBuffer);
    
  }
}