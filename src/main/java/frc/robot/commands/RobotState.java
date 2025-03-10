package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LED.LEDState;
import frc.robot.subsystems.LED.Patterns;

public enum RobotState {
  PREPARE_IDLE(false, false, new LEDState(Color.kPurple, Patterns.SLOW_BLINK)),
  WAIT_IDLE(false, false,  new LEDState(Color.kBlue, Patterns.SOLID)),
  IDLE(false, false,  new LEDState(Color.kBlue, Patterns.SOLID)),
  PREPARE_INVERTED_IDLE(false, true,  new LEDState(Color.kPurple, Patterns.SLOW_BLINK)),
  INVERTED_IDLE(false, true,  new LEDState(Color.kBlue, Patterns.SOLID)),
  DEEP_CLIMB_DEPLOY(true, false,  new LEDState(Color.kYellow, Patterns.SOLID)),
  DEEP_CLIMB_RETRACT(true, false,  new LEDState(Color.kYellowGreen, Patterns.SOLID)),
  DEEP_CLIMB_UNWIND(true, false,  new LEDState(Color.kYellowGreen, Patterns.SOLID)),
  POST_DEEP_CLIMB_DEPLOY(true, false,  new LEDState(Color.kYellowGreen, Patterns.SOLID)),
  DEEP_CLIMB_WAIT(true, false, new LEDState (Color.kGreen, Patterns.SOLID)),
  PREPARE_DEEP_CLIMB(true, false, new LEDState(Color.kYellowGreen, Patterns.SOLID)),
  PREPARE_L1(false, true,  new LEDState(Color.kPurple, Patterns.SLOW_BLINK)),
  WAIT_L1(false, true,  new LEDState(Color.kYellowGreen, Patterns.SOLID)),
  SCORE_L1(false, true,  new LEDState(Color.kGreen, Patterns.SOLID)),
  PREPARE_L2(false, false,  new LEDState(Color.kPurple, Patterns.SLOW_BLINK)),
  WAIT_L2(false, false,  new LEDState(Color.kYellowGreen, Patterns.SOLID)),
  SCORE_L2(false, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  PREPARE_L3(false, false,  new LEDState(Color.kPurple, Patterns.SLOW_BLINK)),
  WAIT_L3(false, false,  new LEDState(Color.kYellowGreen, Patterns.SOLID)),
  SCORE_L3(false, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  PREPARE_L4(false, false,  new LEDState(Color.kPurple, Patterns.SLOW_BLINK)),
  WAIT_L4(false, false,  new LEDState(Color.kYellowGreen, Patterns.SOLID)),
  L4_ELBOW(false, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  PRE_L4(false, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  PRE_HEIGHT_L4(false, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  SCORE_L4(false, false,  new LEDState(Color.kGreen, Patterns.SOLID)),
  CAPPED_L4(false, false,  new LEDState(Color.kYellowGreen, Patterns.SLOW_BLINK)),
  PREPARE_CORAL_STATION(false, false,  new LEDState(Color.kPurple, Patterns.FAST_BLINK)),
  INTAKE_CORAL_STATION(false, false,  new LEDState(Color.kWhite, Patterns.SOLID)),
  PREPARE_INVERTED_CORAL_STATION(false, true,  new LEDState(Color.kPurple, Patterns.FAST_BLINK)),
  INVERTED_INTAKE_CORAL_STATION(false, true,  new LEDState(Color.kWhite, Patterns.SOLID)),
  POST_INVERTED_CORAL_STATION_INTAKE(false, true,  new LEDState(Color.kWhite, Patterns.SOLID)),
  PREPARE_INVERTED_FROM_IDLE(true, true,  new LEDState(Color.kPurple, Patterns.FAST_BLINK)),
  PREPARE_IDLE_FROM_INVERTED(true, true,  new LEDState(Color.kPurple, Patterns.FAST_BLINK)),
  HOMING_STAGE_1_ELEVATOR(false, true, new LEDState(Color.kRed, Patterns.SLOW_BLINK)),
  HOMING_STAGE_2_ELBOW(true, true, new LEDState(Color.kRed, Patterns.SLOW_BLINK)),
  HOMING_STAGE_3_WRIST(true, true,  new LEDState(Color.kRed, Patterns.SLOW_BLINK)),
  PREPARE_HOMING(true, true, new LEDState(Color.kPurple, Patterns.SOLID)),
  PREPARE_REMOVE_ALGAE_LOW(true, false, new LEDState(Color.kBlue, Patterns.SOLID)),
  WAIT_REMOVE_ALGAE_LOW(false, false, new LEDState(Color.kBlue, Patterns.SOLID)),
  REMOVE_ALGAE_LOW(false, false, new LEDState(Color.kBlue, Patterns.SOLID)),
  PREPARE_REMOVE_ALGAE_HIGH(true, false, new LEDState(Color.kBlue, Patterns.SOLID)),
  WAIT_REMOVE_ALGAE_HIGH(false, false, new LEDState(Color.kBlue, Patterns.SOLID)),
  REMOVE_ALGAE_HIGH(false, false, new LEDState(Color.kBlue, Patterns.SOLID)),
  PREPARE_SCORE_ALGAE(false, false, new LEDState(Color.kBlue, Patterns.SOLID)),
  SCORE_ALGAE_WAIT(false, false, new LEDState(Color.kBlue, Patterns.SOLID)),
  SCORE_ALGAE(false, false, new LEDState(Color.kBlue, Patterns.SOLID));


  public final boolean ignoreRequests;
  public final boolean inverted;
  public final LEDState ledState;

  RobotState(boolean ignoreRequests, boolean inverted, LEDState ledState){
    this.ignoreRequests = ignoreRequests;
    this.inverted = inverted;
    this.ledState = ledState;
  }
}
