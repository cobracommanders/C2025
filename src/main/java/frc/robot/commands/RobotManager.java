package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.RobotMode.AlgaeScoreMode;
import frc.robot.commands.RobotMode.CycleMode;
import frc.robot.commands.RobotMode.GameMode;
import frc.robot.commands.RobotMode.IntakeMode;
import frc.robot.FieldConstants;
import frc.robot.FlagManager;
import frc.robot.StateMachine;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climberwheel.ClimberWheelSpeeds;
import frc.robot.subsystems.climberwheel.ClimberWheelState;
import frc.robot.subsystems.climberwheel.ClimberWheelSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainState;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elbow.ElbowState;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorState;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.intakerollers.RollerState;
import frc.robot.subsystems.intakerollers.RollerSubsystem;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RobotManager extends StateMachine<RobotState> {
  private final String name = getName();
  public final ElevatorSubsystem elevator;
  public final ClimberSubsystem climber;
  public final ClimberWheelSubsystem climberwheels;
  public final ManipulatorSubsystem manipulator;
  public final WristSubsystem wrist;
  public final ElbowSubsystem elbow;
  public final RollerSubsystem rollers;
  public final IntakeSubsystem intake;
  public final DrivetrainSubsystem drivetrain;

  public boolean isHeightCapped = true;
  public GameMode currentGameMode = GameMode.CORAL;
  public CycleMode currentCycleMode = CycleMode.REGULAR_CYCLE;
  public IntakeMode currentIntakeMode = IntakeMode.NORMAL;
  public AlgaeScoreMode currentAlgaeScoreMode = AlgaeScoreMode.REGULAR;
  public boolean isInverted = false;
  public Timer timer = new Timer();
  public RobotState overrideState;

  public final FlagManager<RobotFlag> flags = new FlagManager<>("RobotManager", RobotFlag.class);

  public RobotManager() {
    super(RobotState.INVERTED_IDLE);
    this.elevator = ElevatorSubsystem.getInstance();
    this.climber = ClimberSubsystem.getInstance();
    this.climberwheels = ClimberWheelSubsystem.getInstance();
    this.manipulator = ManipulatorSubsystem.getInstance();
    this.wrist = WristSubsystem.getInstance();
    this.elbow = ElbowSubsystem.getInstance();
    this.rollers = RollerSubsystem.getInstance();
    this.intake = IntakeSubsystem.getInstance();
    this.drivetrain = DrivetrainSubsystem.getInstance();
  }

  @Override
  protected void collectInputs() {
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    flags.log();
    RobotState nextState = currentState;
    // if (DriverStation.isDisabled() && DriverStation.isAutonomous()){
    //     nextState = RobotState.INVERTED_IDLE;
    // }
    for (RobotFlag flag : flags.getChecked()) {
      switch (flag) {
        case ALGAE_MODE:
          currentGameMode = GameMode.ALGAE;
          break;
        case CORAL_MODE:
          currentGameMode = GameMode.CORAL;
          break;
        case SUPERCYCLE:
          currentCycleMode = CycleMode.SUPERCYCLE;
          break;
        case REGULAR_CYCLE:
          currentCycleMode = CycleMode.REGULAR_CYCLE;
          break;
        case NORMAL_GROUND_INTAKE:
          currentIntakeMode = IntakeMode.NORMAL;
          break;
        case GROUND_INTAKE_FAILSAFE:
          currentIntakeMode = IntakeMode.FAILSAFE;
          break;
        case NORMAL_ALGAE_SCORE:
          currentAlgaeScoreMode = AlgaeScoreMode.REGULAR;
          break;
        case FRONT_ALGAE_SCORE:
          currentAlgaeScoreMode = AlgaeScoreMode.FRONT;
          break;
        case APPLY_HEIGHT_CAP:
          isHeightCapped = true;
          break;
        case REMOVE_HEIGHT_CAP:
          isHeightCapped = false;
          break;
        case OVERRIDE_STATE:
          currentState = nextState;
          break;
        case IDLE:
          if (!currentState.ignoreRequests) {
            if (RobotMode.getInstance().inAlgaeMode()) {
              if (currentState == RobotState.GROUND_ALGAE_INTAKE && ManipulatorSubsystem.getInstance().hasAlgae()) {
                nextState = RobotState.PREPARE_POST_GROUND_ALGAE_INTAKE;
              } else if (currentState == RobotState.GROUND_ALGAE_INTAKE) {
                nextState = RobotState.PREPARE_IDLE;
              } else if (currentState == RobotState.GROUND_ALGAE_OUTTAKE) {
                nextState = RobotState.PREPARE_IDLE;
              } else {
                nextState = (!currentState.inverted) ? RobotState.PREPARE_IDLE : RobotState.PREPARE_IDLE_FROM_INVERTED;
              }
            }
            else {
              nextState = (!currentState.inverted) ? RobotState.PREPARE_IDLE : RobotState.PREPARE_IDLE_FROM_INVERTED;
            }
          }
          break;
        case INVERTED_IDLE:
          if (!currentState.ignoreRequests) {
            if (RobotMode.getInstance().inCoralMode()) {
              if (currentState == RobotState.INVERTED_INTAKE_CORAL_STATION) {
                nextState = RobotState.POST_INVERTED_CORAL_STATION_INTAKE;
              } else {
                nextState = (currentState.inverted) ? RobotState.PREPARE_INVERTED_IDLE : RobotState.PREPARE_INVERTED_FROM_IDLE;
              }
            }
          }
          break;
        case CORAL_STATION:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            nextState = RobotState.PREPARE_CORAL_STATION;
          }
          break;
        case INVERTED_CORAL_STATION:
          if (!currentState.ignoreRequests && currentState.inverted) {
            nextState = RobotState.PREPARE_INVERTED_CORAL_STATION;
          }
          break;
        case L1:
          if (!currentState.ignoreRequests) {
            nextState = RobotState.PREPARE_L1;
          }
          break;
        case L2:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            nextState = RobotState.PREPARE_L2;
          }
          break;
        case L3:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            nextState = RobotState.PREPARE_L3;
          }
          break;
        case L4:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            nextState = RobotState.PRE_L4;
          }
          break;
        case DEEP_CLIMB:
          if (!currentState.ignoreRequests) {
            nextState = RobotState.PREPARE_DEEP_CLIMB;
          }
          break;
        case FAILSAFE_DEEP_CLIMB:
          nextState = RobotState.FAILSAFE_PREPARE_DEEP_CLIMB;
          break;
        case PROCESSOR:
          if (!currentState.ignoreRequests) {
            nextState = RobotState.PRE_PREPARE_PROCESSOR;
          }
          break;
        case CLIMB_UNWIND:
          nextState = RobotState.DEEP_CLIMB_UNWIND;
          break;
        case CLIMB_IDLE:
          nextState = RobotState.DEEP_CLIMB_WAIT;
          break;
        case CLIMB_RETRACT:
          nextState = RobotState.DEEP_CLIMB_RETRACT;
          break;
        case FAILSAFE_CLIMB_RETRACT:
          nextState = RobotState.FAILSAFE_DEEP_CLIMB_RETRACT;
          break;
        case INTAKE_ALGAE:
          if (currentState == RobotState.WAIT_REMOVE_ALGAE_HIGH){
            nextState = RobotState.REMOVE_ALGAE_HIGH;
          }
          else if (currentState == RobotState.WAIT_REMOVE_ALGAE_LOW){
            nextState = RobotState.REMOVE_ALGAE_LOW;
          }
          break;
        case GROUND_ALGAE_INTAKE:
          if (currentIntakeMode == IntakeMode.NORMAL) {
            if (currentState == RobotState.IDLE){
              nextState = RobotState.PREPARE_GROUND_ALGAE_INTAKE;
            } else if ((currentState == RobotState.POST_GROUND_ALGAE_INTAKE) && ManipulatorSubsystem.getInstance().hasAlgae()) {
              nextState = RobotState.PREPARE_GROUND_ALGAE_OUTTAKE;
            }
          } else if (currentIntakeMode == IntakeMode.FAILSAFE) {
            if (currentState == RobotState.IDLE){
              nextState = RobotState.FAILSAFE_PREPARE_GROUND_ALGAE_INTAKE;
            } else if ((currentState == RobotState.FAILSAFE_POST_GROUND_ALGAE_INTAKE) && ManipulatorSubsystem.getInstance().hasAlgae()) {
              nextState = RobotState.FAILSAFE_PREPARE_GROUND_ALGAE_OUTTAKE;
            }
          }

        case STOP_INTAKE_ALGAE:
          if (currentState == RobotState.REMOVE_ALGAE_HIGH){
            nextState = RobotState.WAIT_REMOVE_ALGAE_HIGH;
          }
          else if (currentState == RobotState.REMOVE_ALGAE_LOW){
            nextState = RobotState.WAIT_REMOVE_ALGAE_LOW;
          }
          else if(currentState == RobotState.SCORE_ALGAE){
            nextState = RobotState.WAIT_REMOVE_ALGAE_HIGH; 
          }
          break;
        case ALGAE_HIGH:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            if ((currentState == RobotState.SCORE_L4) || (currentState == RobotState.SCORE_L3) || (currentState == RobotState.SCORE_L2)) {
              nextState = RobotState.PRE_SUPERCYCLE_HIGH_ALGAE;
            } else {
              nextState = RobotState.PREPARE_REMOVE_ALGAE_HIGH;
            }
          }
          break;
        case ALGAE_LOW:
          if (!currentState.ignoreRequests && !currentState.inverted) {
            if ((currentState == RobotState.SCORE_L4) || (currentState == RobotState.SCORE_L3) || (currentState == RobotState.SCORE_L2)) {
              nextState = RobotState.PRE_SUPERCYCLE_LOW_ALGAE;
            } else {
              nextState = RobotState.PREPARE_REMOVE_ALGAE_LOW;
            }
          }
          break;
        case HOMING:
          nextState = RobotState.HOMING_STAGE_1_ELEVATOR;
          break;
        case SCORE:
          switch (nextState) {
            case WAIT_L1:
              nextState = RobotState.SCORE_L1;
              break;
            case WAIT_L2:
              nextState = RobotState.SCORE_L2;
              break;
            case WAIT_L3:
              nextState = RobotState.SCORE_L3;
              break;
            case L4_ELBOW:
              nextState = RobotState.SCORE_L4;
              break;
            case WAIT_L4:
              nextState = RobotState.SCORE_L4;
              break;
            case CAPPED_L4:
              nextState = RobotState.SCORE_L4;
              break;
            case SCORE_ALGAE_WAIT:
                if (currentAlgaeScoreMode == AlgaeScoreMode.REGULAR) {
                  nextState = RobotState.SCORE_ALGAE;
                } else {
                  nextState = RobotState.FRONT_SCORE_ALGAE;
                }
              break;
            case WAIT_PROCESSOR:
              nextState = RobotState.SCORE_PROCESSOR;
              break;
            default:
              break;
          }
      }
    }
    switch (currentState) {
      case WAIT_L2:
      case IDLE:
      case WAIT_IDLE:
      case WAIT_L1:
      case INTAKE_CORAL_STATION:
      case INVERTED_IDLE:
      case WAIT_L3:
      case PRE_HEIGHT_L4:
      case DEEP_CLIMB_RETRACT:
      case FAILSAFE_DEEP_CLIMB_RETRACT:
      case DEEP_CLIMB_UNWIND:
      case WAIT_PROCESSOR:
        break;

      case DEEP_CLIMB_UNLATCH:
        if(timeout(1)){
          nextState = RobotState.DEEP_CLIMB_DEPLOY;
          //climber.setDeployConfig();
        } 
        break;
      case DEEP_CLIMB_DEPLOY:
        if(ClimberSubsystem.getInstance().climberDeployed()){
          if (timeout(0.4)) {
            nextState = RobotState.DEEP_CLIMB_WAIT;
          }
        } 
        break;
      case FAILSAFE_DEEP_CLIMB_DEPLOY:
        if(ClimberSubsystem.getInstance().climberDeployed()){
          if (timeout(0.4)) {
            nextState = RobotState.FAILSAFE_DEEP_CLIMB_WAIT;
          }
        } 
        break;
      case PRE_L4:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.PREPARE_L4;
        }
        break;
      case PREPARE_L1:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_L1;
        }
        break;
      case PREPARE_L2:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_L2;
        }
        break;
      case PREPARE_L3:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_L3;
      }
        break;
      case CAPPED_L4:
        if (!isHeightCapped) {
          nextState = RobotState.PREPARE_L4;
        }
        break;
      case PREPARE_L4:
        if(isHeightCapped) {
          nextState = RobotState.CAPPED_L4;
        } else if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.L4_ELBOW;
        }
        break;
      case WAIT_L4:
        if(isHeightCapped) {
          nextState = RobotState.CAPPED_L4;
        }
        break;
      case L4_ELBOW:
        if(isHeightCapped) {
          nextState = RobotState.CAPPED_L4;
        } else if (elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_L4;
        }
        break;
      case PREPARE_REMOVE_ALGAE_HIGH:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && intake.atGoal()) {
          nextState = RobotState.REMOVE_ALGAE_HIGH;
        }
        break;
      case PREPARE_REMOVE_ALGAE_LOW:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.WAIT_REMOVE_ALGAE_LOW;
        }
        break;
      case PRE_PREPARE_PROCESSOR:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && intake.atGoal()) {
          nextState = RobotState.PREPARE_PROCESSOR;
        }
        break;
      case PREPARE_PROCESSOR:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && intake.atGoal()) {
          nextState = RobotState.WAIT_PROCESSOR;
        }
        break;
      case PREPARE_SCORE_ALGAE:
        if(isHeightCapped) {
          nextState = RobotState.REMOVE_ALGAE_HIGH;
        } else if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          if (currentAlgaeScoreMode == AlgaeScoreMode.REGULAR){
            nextState = RobotState.SCORE_ALGAE_WAIT;
          } else {
            nextState = RobotState.PRE_FRONT_SCORE_ALGAE;
          }
        }
        break;
      case SCORE_ALGAE_WAIT:
        if(isHeightCapped) {
          nextState = RobotState.REMOVE_ALGAE_HIGH;
        }
        break;
      case PRE_FRONT_SCORE_ALGAE:
        if(elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.FRONT_SCORE_ALGAE_WAIT;
        }
        break;
      case FRONT_SCORE_ALGAE_WAIT:
        if(isHeightCapped) {
          nextState = RobotState.REMOVE_ALGAE_HIGH;
        }
        break;
      case WAIT_REMOVE_ALGAE_LOW:
        if(!isHeightCapped){
          nextState = RobotState.PREPARE_SCORE_ALGAE;
        }
        break;
      case WAIT_REMOVE_ALGAE_HIGH:
        if(!isHeightCapped){
          nextState = RobotState.PREPARE_SCORE_ALGAE;
        }
        break;
      case PREPARE_CORAL_STATION:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.INTAKE_CORAL_STATION;
        }
        break;
      case PREPARE_INVERTED_CORAL_STATION:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.INVERTED_INTAKE_CORAL_STATION;
        }
        break;
      case INVERTED_INTAKE_CORAL_STATION:
        if (ManipulatorSubsystem.getInstance().hasCoral() && timeout(0.6) && DriverStation.isAutonomous()) {
          nextState = RobotState.POST_INVERTED_CORAL_STATION_INTAKE;
        }
        break;
      case GROUND_ALGAE_INTAKE:
        if (ManipulatorSubsystem.getInstance().hasAlgae() && timeout(0.6)) {
          nextState = RobotState.POST_GROUND_ALGAE_INTAKE;
        }
        break;
      case GROUND_ALGAE_OUTTAKE:
        if (timeout(3)) {
          nextState = RobotState.IDLE;
        }
        break;
      case FAILSAFE_GROUND_ALGAE_INTAKE:
        if (ManipulatorSubsystem.getInstance().hasAlgae() && timeout(0.6)) {
          nextState = RobotState.FAILSAFE_POST_GROUND_ALGAE_INTAKE;
        }
        break;
      case FAILSAFE_GROUND_ALGAE_OUTTAKE:
        if (timeout(3)) {
          nextState = RobotState.IDLE;
        }
        break;
      case PREPARE_GROUND_ALGAE_INTAKE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && intake.atGoal()) {
          nextState = RobotState.GROUND_ALGAE_INTAKE;
        }
        break;
      case PREPARE_GROUND_ALGAE_OUTTAKE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && intake.atGoal()) {
          nextState = RobotState.GROUND_ALGAE_OUTTAKE;
        }
        break;
      case PREPARE_POST_GROUND_ALGAE_INTAKE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && intake.atGoal()) {
          nextState = RobotState.POST_GROUND_ALGAE_INTAKE;
        }
        break;
      case FAILSAFE_PREPARE_GROUND_ALGAE_INTAKE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && intake.atGoal()) {
          nextState = RobotState.FAILSAFE_GROUND_ALGAE_INTAKE;
        }
        break;
      case FAILSAFE_PREPARE_GROUND_ALGAE_OUTTAKE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && intake.atGoal()) {
          nextState = RobotState.FAILSAFE_GROUND_ALGAE_OUTTAKE;
        }
        break;
      case FAILSAFE_PREPARE_POST_GROUND_ALGAE_INTAKE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && intake.atGoal()) {
          nextState = RobotState.FAILSAFE_POST_GROUND_ALGAE_INTAKE;
        }
        break;
      case POST_INVERTED_CORAL_STATION_INTAKE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.PREPARE_INVERTED_IDLE;
        }
        break;
      case PREPARE_DEEP_CLIMB:
        if (elbow.atGoal() && elevator.atGoal() && wrist.atGoal()) {
          nextState = RobotState.DEEP_CLIMB_DEPLOY;
        }
        break;
      case FAILSAFE_PREPARE_DEEP_CLIMB:
        nextState = RobotState.FAILSAFE_DEEP_CLIMB_DEPLOY;
        break;
      case PREPARE_IDLE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.IDLE;
        }
        break;
      case PREPARE_INVERTED_IDLE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.INVERTED_IDLE;
        }
        break;
      case SCORE_L1:
        if ((timeout(1) && DriverStation.isTeleop()) || (timeout(0.35) && DriverStation.isAutonomous())) {
          nextState = RobotState.PREPARE_INVERTED_FROM_IDLE;
        }
        break;
      case SCORE_L2:
        if ((timeout(0.5) && DriverStation.isTeleop()) || (timeout(0.35) && DriverStation.isAutonomous())) {
          if (currentCycleMode == CycleMode.SUPERCYCLE) {
            if (FieldConstants.getInstance().isNearHighAlgae()) {
              nextState = RobotState.PRE_SUPERCYCLE_HIGH_ALGAE;
              currentGameMode = GameMode.ALGAE;
            } else {
              nextState = RobotState.PRE_SUPERCYCLE_LOW_ALGAE;
              currentGameMode = GameMode.ALGAE;
            }
          // } else if (currentCycleMode == CycleMode.SUPERCYCLE && DriverStation.isAutonomous()) {
          //   nextState = RobotState.IDLE;
          //   currentGameMode = GameMode.ALGAE;
          } else {
            nextState = RobotState.PREPARE_INVERTED_FROM_IDLE;
          }
        }
        break;
      case SCORE_L3:
        if ((timeout(0.5) && DriverStation.isTeleop()) || (timeout(0.35) && DriverStation.isAutonomous())) {
          if (currentCycleMode == CycleMode.SUPERCYCLE) {
            if (FieldConstants.getInstance().isNearHighAlgae()) {
              nextState = RobotState.PRE_SUPERCYCLE_HIGH_ALGAE;
              currentGameMode = GameMode.ALGAE;
            } else {
              nextState = RobotState.PRE_SUPERCYCLE_LOW_ALGAE;
              currentGameMode = GameMode.ALGAE;
            }
          } else {
            nextState = RobotState.PREPARE_INVERTED_FROM_IDLE;
          }
        }
        break;
      case SCORE_L4:
        if ((timeout(0.75) && DriverStation.isTeleop()) || (timeout(0.35) && DriverStation.isAutonomous())) {
          if (currentCycleMode == CycleMode.SUPERCYCLE) {
            if (FieldConstants.getInstance().isNearHighAlgae()) {
              nextState = RobotState.PRE_SUPERCYCLE_HIGH_ALGAE;
              currentGameMode = GameMode.ALGAE;
            } else {
              nextState = RobotState.PRE_SUPERCYCLE_LOW_ALGAE;
              currentGameMode = GameMode.ALGAE;
            }
          // } else if (currentCycleMode == CycleMode.SUPERCYCLE && DriverStation.isAutonomous()) {
          //   nextState = RobotState.IDLE;
          //   currentGameMode = GameMode.ALGAE;
          } else {
            nextState = RobotState.PREPARE_INVERTED_FROM_IDLE;
          }
        }
        break;
      case REMOVE_ALGAE_HIGH:
        if(!isHeightCapped && DriverStation.isTeleopEnabled()) {
          nextState = RobotState.PREPARE_SCORE_ALGAE;
        } else if (!isHeightCapped && timeout(1)) {
          nextState = RobotState.PREPARE_SCORE_ALGAE;
        }
        break;
      case REMOVE_ALGAE_LOW:
      if(!isHeightCapped && DriverStation.isTeleopEnabled()) {
        nextState = RobotState.PREPARE_SCORE_ALGAE;
      } else if (!isHeightCapped && timeout(1)) {
        nextState = RobotState.PREPARE_SCORE_ALGAE;
      }
        break;
      case POST_GROUND_ALGAE_INTAKE:
        if(!isHeightCapped) {
          nextState = RobotState.PREPARE_SCORE_ALGAE;
        }
        break;
      case FAILSAFE_POST_GROUND_ALGAE_INTAKE:
        if(!isHeightCapped) {
          nextState = RobotState.PREPARE_SCORE_ALGAE;
        }
        break;
      case SCORE_ALGAE:
        if ((timeout(3) && DriverStation.isTeleop() || (timeout(1.5) && DriverStation.isAutonomous()))) {
          nextState = RobotState.PREPARE_IDLE;
        }
        break;
      case FRONT_SCORE_ALGAE:
        if ((timeout(3) && DriverStation.isTeleop() || (timeout(1.5) && DriverStation.isAutonomous()))) {
          nextState = RobotState.PREPARE_IDLE;
        }
        break;
      case SCORE_PROCESSOR:
        if ((timeout(3) && DriverStation.isTeleop() || (timeout(1) && DriverStation.isAutonomous()))) {
          nextState = RobotState.PREPARE_IDLE;
        }
        break;
      case PRE_SUPERCYCLE_HIGH_ALGAE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && timeout(0.5)) {
          nextState = RobotState.PREPARE_REMOVE_ALGAE_HIGH;
        }
        break;
      case PRE_SUPERCYCLE_LOW_ALGAE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal() && timeout(0.5)) {
          nextState = RobotState.PREPARE_REMOVE_ALGAE_LOW;
        }
        break;
      case PREPARE_INVERTED_FROM_IDLE:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.PREPARE_INVERTED_IDLE;
        }
        break;
      case PREPARE_IDLE_FROM_INVERTED:
        if (elevator.atGoal() && elbow.atGoal() && wrist.atGoal()) {
          nextState = RobotState.PREPARE_IDLE;
        }
        break;
      case PREPARE_HOMING:
          if (DriverStation.isEnabled()) {
            nextState = RobotState.HOMING_STAGE_1_ELEVATOR;
          }
        break;
      case HOMING_STAGE_1_ELEVATOR:
        if (elevator.isIdle()) {
          nextState = RobotState.HOMING_STAGE_2_ELBOW;
        }
        break;
      case HOMING_STAGE_2_ELBOW:
        if (elbow.isIdle()) {//change to isIdle
          nextState = RobotState.HOMING_STAGE_3_WRIST;
        }
        break;
      case HOMING_STAGE_3_WRIST:
        if (wrist.isIdle()) {
          nextState = RobotState.INVERTED_IDLE;
        }
        break;
      // case DEEP_CLIMB_RETRACT:
      //   // if (ClimberWheelSpeeds.INTAKE_CAGE == ClimberWheelSpeeds.STATIC_INTAKE_CAGE) {
      //   //   ClimberWheelSubsystem.getInstance().hasCage();
      //   // }
      //   if (timeout(1.5)){
      //     nextState = RobotState.DEEP_CLIMB_WAIT;
      //   }
      //   break;
      // case DEEP_CLIMB_UNWIND:
      //   // if (ClimberWheelSpeeds.INTAKE_CAGE == ClimberWheelSpeeds.STATIC_INTAKE_CAGE) {
      //   //   ClimberWheelSubsystem.getInstance().hasCage();
      //   // }  
      //   if (timeout(1.5)){
      //     nextState = RobotState.DEEP_CLIMB_WAIT;
      //   }
      //   break;
      case DEEP_CLIMB_WAIT:
        if (ClimberWheelSpeeds.INTAKE_CAGE == ClimberWheelSpeeds.STATIC_INTAKE_CAGE) {
          ClimberWheelSubsystem.getInstance().hasCage();
        }
        if (ClimberWheelSubsystem.getInstance().hasCage()) {
          nextState = RobotState.DEEP_CLIMB_RETRACT;
        }
        break;
      case FAILSAFE_DEEP_CLIMB_WAIT:
        if (ClimberWheelSpeeds.INTAKE_CAGE == ClimberWheelSpeeds.STATIC_INTAKE_CAGE) {
          ClimberWheelSubsystem.getInstance().hasCage();
        }
        if (ClimberWheelSubsystem.getInstance().hasCage()) {
          nextState = RobotState.FAILSAFE_DEEP_CLIMB_RETRACT;
        }
        break;
    }
    DogLog.log(name + "/AtGoal", elevator.atGoal() && elbow.atGoal() && wrist.atGoal());
    DogLog.log(name + "/isCoralMode", this.currentGameMode == GameMode.CORAL);
    flags.clear();
    return nextState;
  };
  

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
          case IDLE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
            intake.setState(IntakeState.IDLE);
            rollers.setState(RollerState.IDLE);
          }
          case PRE_L4 -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
          }
          case PREPARE_L1 -> {
            elevator.setState(ElevatorState.L1);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.PREPARE_L1);
            wrist.setState(WristState.L1);
            elbow.setState(ElbowState.L1);
          }
          case SCORE_L1 -> {
            elevator.setState(ElevatorState.L1);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L1);
            wrist.setState(WristState.L1);
            elbow.setState(ElbowState.L1);
          }
          case PREPARE_L2 -> {
            elevator.setState(ElevatorState.L2);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.PREPARE_L2);
            wrist.setState(WristState.L2);
            elbow.setState(ElbowState.L2);
          }
          case SCORE_L2 -> {
            elevator.setState(ElevatorState.L2);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L2);
            wrist.setState(WristState.L2);
            elbow.setState(ElbowState.L2);
          }
          case PREPARE_L3 -> {
            elevator.setState(ElevatorState.L3);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.PREPARE_L3);
            wrist.setState(WristState.L3);
            elbow.setState(ElbowState.L3);
          }
          case SCORE_L3 -> {
            elevator.setState(ElevatorState.L3);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L3);
            wrist.setState(WristState.L3);
            elbow.setState(ElbowState.L3);
          }
          case PREPARE_L4 -> {
            elevator.setState(ElevatorState.L4);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.PREPARE_L4);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.L4);
          }
          case SCORE_L4 -> {
            elevator.setState(ElevatorState.L4_MAX);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.L4);
            wrist.setState(WristState.L4_WRIST);
            elbow.setState(ElbowState.L4);
          }
          case PREPARE_CORAL_STATION -> {
            elevator.setState(ElevatorState.CORAL_STATION);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.CORAL_STATION);
            elbow.setState(ElbowState.CORAL_STATION);
          }
          case INTAKE_CORAL_STATION -> {
            elevator.setState(ElevatorState.CORAL_STATION);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_CORAL);
            wrist.setState(WristState.CORAL_STATION);
            elbow.setState(ElbowState.CORAL_STATION);
          }
          case POST_INVERTED_CORAL_STATION_INTAKE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.INVERTED_CORAL_STATION);
            elbow.setState(ElbowState.INVERTED_CORAL_STATION);
          }
          case PREPARE_INVERTED_CORAL_STATION -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.INVERTED_CORAL_STATION);
            elbow.setState(ElbowState.INVERTED_CORAL_STATION);
          }
          case PREPARE_GROUND_ALGAE_INTAKE -> {
            elevator.setState(ElevatorState.GROUND_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.GROUND_ALGAE_INTAKE);
            elbow.setState(ElbowState.IDLE);
            intake.setState(IntakeState.INTAKE);
            rollers.setState(RollerState.IDLE);
          }
          case PREPARE_GROUND_ALGAE_OUTTAKE -> {
            elevator.setState(ElevatorState.GROUND_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.GROUND_ALGAE_INTAKE);
            elbow.setState(ElbowState.GROUND_ALGAE_INTAKE);
            intake.setState(IntakeState.OUTTAKE);
            rollers.setState(RollerState.IDLE);
          }
          case FAILSAFE_PREPARE_GROUND_ALGAE_INTAKE -> {
            elevator.setState(ElevatorState.FAILSAFE_GROUND_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.FAILSAFE_GROUND_ALGAE_INTAKE);
            elbow.setState(ElbowState.FAILSAFE_GROUND_ALGAE_INTAKE);
            intake.setState(IntakeState.IDLE);
            rollers.setState(RollerState.IDLE);
          }
          case FAILSAFE_PREPARE_GROUND_ALGAE_OUTTAKE -> {
            elevator.setState(ElevatorState.FAILSAFE_GROUND_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.PROCESSOR);
            elbow.setState(ElbowState.PROCESSOR);
            intake.setState(IntakeState.IDLE);
            rollers.setState(RollerState.IDLE);
          }
          case INVERTED_INTAKE_CORAL_STATION -> {
            elevator.setState(ElevatorState.INVERTED_CORAL_STATION);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_CORAL);
            wrist.setState(WristState.INVERTED_CORAL_STATION);
            elbow.setState(ElbowState.INVERTED_CORAL_STATION);
          }
          case GROUND_ALGAE_INTAKE -> {
            elevator.setState(ElevatorState.GROUND_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_GROUND_ALGAE);
            wrist.setState(WristState.GROUND_ALGAE_INTAKE);
            elbow.setState(ElbowState.GROUND_ALGAE_INTAKE);
            intake.setState(IntakeState.INTAKE);
            rollers.setState(RollerState.INTAKE);
          }
          case GROUND_ALGAE_OUTTAKE -> {
            elevator.setState(ElevatorState.GROUND_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.OUTTAKE_ALGAE);
            wrist.setState(WristState.GROUND_ALGAE_INTAKE);
            elbow.setState(ElbowState.GROUND_ALGAE_INTAKE);
            intake.setState(IntakeState.OUTTAKE);
            rollers.setState(RollerState.OUTTAKE);
          }
          case FAILSAFE_GROUND_ALGAE_INTAKE -> {
            elevator.setState(ElevatorState.FAILSAFE_GROUND_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_GROUND_ALGAE_FAILSAFE);
            wrist.setState(WristState.FAILSAFE_GROUND_ALGAE_INTAKE);
            elbow.setState(ElbowState.FAILSAFE_GROUND_ALGAE_INTAKE);
            intake.setState(IntakeState.IDLE);
            rollers.setState(RollerState.IDLE);
          }
          case FAILSAFE_GROUND_ALGAE_OUTTAKE -> {
            elevator.setState(ElevatorState.PROCESSOR);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.OUTTAKE_ALGAE);
            wrist.setState(WristState.PROCESSOR);
            elbow.setState(ElbowState.PROCESSOR);
            intake.setState(IntakeState.IDLE);
            rollers.setState(RollerState.IDLE);
          }
          case PREPARE_DEEP_CLIMB -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            climberwheels.setState(ClimberWheelState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
          }

          case FAILSAFE_PREPARE_DEEP_CLIMB -> {
            climber.setState(ClimberState.IDLE);
            climberwheels.setState(ClimberWheelState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
          }

          case DEEP_CLIMB_UNLATCH -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.DEEP_CLIMB_UNLATCH);
            climberwheels.setState(ClimberWheelState.IDLE);
            wrist.setState(WristState.CAGE_FLIP);
            elbow.setState(ElbowState.CAGE_FLIP);
            intake.setState(IntakeState.CAGE_FLIP);
            rollers.setState(RollerState.IDLE);
          }
          
          case DEEP_CLIMB_DEPLOY -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.DEEP_CLIMB_DEPLOY);
            climberwheels.setState(ClimberWheelState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.CAGE_FLIP);
            elbow.setState(ElbowState.CAGE_FLIP);
            intake.setState(IntakeState.CAGE_FLIP);
            rollers.setState(RollerState.IDLE);
          }

          case FAILSAFE_DEEP_CLIMB_DEPLOY -> {
            climber.setState(ClimberState.DEEP_CLIMB_DEPLOY);
            climberwheels.setState(ClimberWheelState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            intake.setState(IntakeState.CAGE_FLIP);
            rollers.setState(RollerState.IDLE);
          }

          case DEEP_CLIMB_WAIT -> {
            climber.setRetractConfig();
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.DEEP_CLIMB_WAIT);
            climberwheels.setState(ClimberWheelState.INTAKE_CAGE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.CAGE_FLIP);
            elbow.setState(ElbowState.CAGE_FLIP);
            intake.setState(IntakeState.CAGE_FLIP);
            rollers.setState(RollerState.IDLE);
          }

          case FAILSAFE_DEEP_CLIMB_WAIT -> {
            climber.setRetractConfig();
            climber.setState(ClimberState.DEEP_CLIMB_WAIT);
            climberwheels.setState(ClimberWheelState.INTAKE_CAGE);
            manipulator.setState(ManipulatorState.IDLE);
            intake.setState(IntakeState.CAGE_FLIP);
            rollers.setState(RollerState.IDLE);
          }

          case DEEP_CLIMB_RETRACT -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.DEEP_CLIMB_RETRACT);
            climberwheels.setState(ClimberWheelState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.CAGE_FLIP);
            elbow.setState(ElbowState.CAGE_FLIP);
            intake.setState(IntakeState.CAGE_FLIP);
            rollers.setState(RollerState.IDLE);
          }

          case FAILSAFE_DEEP_CLIMB_RETRACT -> {
            climber.setState(ClimberState.DEEP_CLIMB_RETRACT);
            climberwheels.setState(ClimberWheelState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            intake.setState(IntakeState.CAGE_FLIP);
            rollers.setState(RollerState.IDLE);
          }

          case DEEP_CLIMB_UNWIND -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.DEEP_CLIMB_UNWIND);
            climberwheels.setState(ClimberWheelState.INTAKE_CAGE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.CAGE_FLIP);
            elbow.setState(ElbowState.CAGE_FLIP);
            intake.setState(IntakeState.CAGE_FLIP);
            rollers.setState(RollerState.IDLE);
          }

          case CAPPED_L4 -> {
            elevator.setState(ElevatorState.CAPPED_L4);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.CAPPED_L4);
          }

          case L4_ELBOW -> {
            elevator.setState(ElevatorState.L4_MAX);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.AFTER_INTAKE);
            wrist.setState(WristState.L4_WRIST);
            elbow.setState(ElbowState.L4_ELBOW);
          }

          case PREPARE_IDLE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
            intake.setState(IntakeState.IDLE);
            rollers.setState(RollerState.IDLE);
          }

          case PREPARE_IDLE_FROM_INVERTED -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.INVERTED_IDLE);
            elbow.setState(ElbowState.INVERTED_IDLE);
          }

          case PREPARE_INVERTED_FROM_IDLE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
            intake.setState(IntakeState.IDLE);
            rollers.setState(RollerState.IDLE);
          }

          case PREPARE_INVERTED_IDLE -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            wrist.setState(WristState.INVERTED_IDLE);
            elbow.setState(ElbowState.INVERTED_IDLE);
          }
          case PREPARE_REMOVE_ALGAE_HIGH -> {
            elevator.setState(ElevatorState.HIGH_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            elbow.setState(ElbowState.HIGH_ALGAE);
          }
          case PREPARE_REMOVE_ALGAE_LOW -> {
            elevator.setState(ElevatorState.LOW_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            elbow.setState(ElbowState.LOW_ALGAE);
          }
          case PRE_PREPARE_PROCESSOR -> {
            elevator.setState(ElevatorState.IDLE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            elbow.setState(ElbowState.IDLE);
            intake.setState(IntakeState.PROCESSOR);
            rollers.setState(RollerState.IDLE);
          }
          case PREPARE_PROCESSOR -> {
            elevator.setState(ElevatorState.PROCESSOR);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.PROCESSOR);
            elbow.setState(ElbowState.PROCESSOR);
            intake.setState(IntakeState.PROCESSOR);
            rollers.setState(RollerState.IDLE);
          }

          case WAIT_REMOVE_ALGAE_HIGH -> {
            elevator.setState(ElevatorState.HIGH_ALGAE);
            climber.setState(ClimberState.IDLE);
            elbow.setState(ElbowState.HIGH_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
          }
          case WAIT_REMOVE_ALGAE_LOW -> {
            elevator.setState(ElevatorState.LOW_ALGAE);
            climber.setState(ClimberState.IDLE);
            elbow.setState(ElbowState.LOW_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
          }
          case WAIT_PROCESSOR -> {
            elevator.setState(ElevatorState.PROCESSOR);
            climber.setState(ClimberState.IDLE);
            elbow.setState(ElbowState.PROCESSOR);
            wrist.setState(WristState.PROCESSOR);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            intake.setState(IntakeState.PROCESSOR);
            rollers.setState(RollerState.IDLE);
          }
          case PRE_SUPERCYCLE_HIGH_ALGAE -> {
            elevator.setState(ElevatorState.HIGH_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
            intake.setState(IntakeState.IDLE);
            rollers.setState(RollerState.IDLE);
          }
          case PRE_SUPERCYCLE_LOW_ALGAE -> {
            elevator.setState(ElevatorState.LOW_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.IDLE);
            elbow.setState(ElbowState.IDLE);
            intake.setState(IntakeState.IDLE);
            rollers.setState(RollerState.IDLE);
          }

          case REMOVE_ALGAE_HIGH -> {
            elevator.setState(ElevatorState.HIGH_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            elbow.setState(ElbowState.HIGH_ALGAE);
          }
          case REMOVE_ALGAE_LOW -> {
            elevator.setState(ElevatorState.LOW_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            elbow.setState(ElbowState.LOW_ALGAE);
          }
          case SCORE_PROCESSOR -> {
            elevator.setState(ElevatorState.PROCESSOR);
            climber.setState(ClimberState.IDLE);
            elbow.setState(ElbowState.PROCESSOR);
            wrist.setState(WristState.PROCESSOR);
            manipulator.setState(ManipulatorState.SCORE_PROCESSOR);
            intake.setState(IntakeState.PROCESSOR);
            rollers.setState(RollerState.PROCESSOR);
          }
          
          case PREPARE_POST_GROUND_ALGAE_INTAKE -> {
            elevator.setState(ElevatorState.LOW_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            elbow.setState(ElbowState.LOW_ALGAE);
            intake.setState(IntakeState.INTAKE);
            rollers.setState(RollerState.IDLE);
          }

          case POST_GROUND_ALGAE_INTAKE -> {
            elevator.setState(ElevatorState.LOW_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            elbow.setState(ElbowState.LOW_ALGAE);
            intake.setState(IntakeState.IDLE);
            rollers.setState(RollerState.IDLE);
          }

          case FAILSAFE_PREPARE_POST_GROUND_ALGAE_INTAKE -> {
            elevator.setState(ElevatorState.LOW_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.PROCESSOR);
            elbow.setState(ElbowState.PROCESSOR);
            intake.setState(IntakeState.IDLE);
            rollers.setState(RollerState.IDLE);
          }

          case FAILSAFE_POST_GROUND_ALGAE_INTAKE -> {
            elevator.setState(ElevatorState.LOW_ALGAE);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.INTAKE_ALGAE);
            elbow.setState(ElbowState.LOW_ALGAE);
            intake.setState(IntakeState.IDLE);
            rollers.setState(RollerState.IDLE);
          }

          case PREPARE_SCORE_ALGAE -> {
            elevator.setState(ElevatorState.L4);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.PRE_ALGAE_SCORE);
            elbow.setState(ElbowState.PRE_SCORE_ALGAE);
          }

          case SCORE_ALGAE_WAIT -> {
            elevator.setState(ElevatorState.L4_MAX);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.PRE_ALGAE_SCORE);
            elbow.setState(ElbowState.L4);
          }


          case SCORE_ALGAE -> {
            elevator.setState(ElevatorState.L4_MAX);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.SCORE_ALGAE);
            elbow.setState(ElbowState.L4);
          }

          case PRE_FRONT_SCORE_ALGAE -> {
            elevator.setState(ElevatorState.L4_MAX);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.PRE_ALGAE_SCORE);
            elbow.setState(ElbowState.L4);
          }

          case FRONT_SCORE_ALGAE_WAIT -> {
            elevator.setState(ElevatorState.L4_MAX);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.INTAKE_ALGAE);
            wrist.setState(WristState.FRONT_ALGAE_SCORE);
            elbow.setState(ElbowState.L4);
          }

          case FRONT_SCORE_ALGAE -> {
            elevator.setState(ElevatorState.L4_MAX);
            climber.setState(ClimberState.IDLE);
            manipulator.setState(ManipulatorState.SCORE_ALGAE);
            wrist.setState(WristState.FRONT_ALGAE_SCORE);
            elbow.setState(ElbowState.L4);
          }
          
          case HOMING_STAGE_1_ELEVATOR -> {
            elevator.setState(ElevatorState.HOME_ELEVATOR);
            wrist.setState(WristState.DISABLED);
            elbow.setState(ElbowState.DISABLED);
          }
          case HOMING_STAGE_2_ELBOW -> {
            elbow.setState(ElbowState.HOME_ELBOW);
            wrist.setState(WristState.DISABLED);
          }
          case HOMING_STAGE_3_WRIST -> {
            wrist.setState(WristState.HOME_WRIST);
          }
          case WAIT_L2 -> {
            manipulator.setState(ManipulatorState.PRE_SCORE);
          }
          case WAIT_L3 -> {
            manipulator.setState(ManipulatorState.PRE_SCORE);
          }
          case WAIT_L4 -> {
            manipulator.setState(ManipulatorState.IDLE);
            elbow.setState(ElbowState.L4_ELBOW);
            wrist.setState(WristState.L4_WRIST);
            elevator.setState(ElevatorState.L4_MAX);
          }

          case INVERTED_IDLE -> {
            elevator.setState(ElevatorState.IDLE);
            elbow.setState(ElbowState.INVERTED_IDLE);
            wrist.setState(WristState.INVERTED_IDLE);
            manipulator.setState(ManipulatorState.IDLE);
            //wrist.syncEncoder();
          }
          case 
            WAIT_IDLE, 
            WAIT_L1,
            PRE_HEIGHT_L4,
            PREPARE_HOMING -> {}
          }
      }

  @Override
  public void periodic() {
    super.periodic(); 
    DogLog.log(name + "/is Coral Mode", RobotMode.getInstance().inCoralMode());
    DogLog.log(name + "/Is capped", isHeightCapped);
    if (RobotManager.getInstance().getState() == RobotState.SCORE_ALGAE && timeout(0.165)) {
      manipulator.setState(ManipulatorState.SCORE_ALGAE);
    }
    //DogLog.log(getName() + "Active Command", elevator.getCurrentCommand().toString());
  }

  public void prepareIdleRequest() {
    flags.check(RobotFlag.IDLE);
  }

  public void prepareInvertedIdleRequest(){
    flags.check(RobotFlag.INVERTED_IDLE);
  }

  public void prepareL1Request() {
    flags.check(RobotFlag.L1);
  }

  public void prepareL2Request() {
    flags.check(RobotFlag.L2);
  }

  public void prepareL3Request() {
    flags.check(RobotFlag.L3);
  }
  
  public void prepareL4Request() {
    flags.check(RobotFlag.L4);
  }

  public void prepareAlgaeHighRequest() {
    flags.check(RobotFlag.ALGAE_HIGH);
  }

  public void prepareAlgaeLowRequest() {
    flags.check(RobotFlag.ALGAE_LOW);
  }
  
  public void prepareDeepClimbRequest() {
    flags.check(RobotFlag.DEEP_CLIMB);
  }

  public void setProcessorRequest() {
    flags.check(RobotFlag.PROCESSOR);
  }

  public void prepareCoralStationRequest() {
    flags.check(RobotFlag.CORAL_STATION);
  }

  public void prepareInvertedCoralStationRequest() {
    flags.check(RobotFlag.INVERTED_CORAL_STATION);
  }

  public void intakeAlgaeRequest(){
    flags.check(RobotFlag.GROUND_ALGAE_INTAKE);
  }

  public void stopIntakeAlgaeRequest(){
    flags.check(RobotFlag.STOP_INTAKE_ALGAE);
  }

  public void intakeGroundAlgaeRequest(){
    flags.check(RobotFlag.GROUND_ALGAE_INTAKE);
  }

  public void scoreRequest() {
    flags.check(RobotFlag.SCORE);
  }

  public void algaeScoreRequest() {
    flags.check(RobotFlag.SCORE);
  }

  public void climbRequest(){
    flags.check(RobotFlag.DEEP_CLIMB);
  }

  public void failsafeClimbRequest(){
    flags.check(RobotFlag.FAILSAFE_DEEP_CLIMB);
  }

  public void climbUnwindRequest(){
    flags.check(RobotFlag.CLIMB_UNWIND);
  }

  public void climbIdleRequest(){
    flags.check(RobotFlag.CLIMB_IDLE);
  }

  public void climbRetractRequest(){
    flags.check(RobotFlag.CLIMB_RETRACT);
  }

  public void failsafeClimbRetractRequest(){
    flags.check(RobotFlag.FAILSAFE_CLIMB_RETRACT);
  }

  public void frontAlgaeScoreRequest(){
    flags.check(RobotFlag.FRONT_ALGAE_SCORE);
  }

  public void normalAlgaeScoreRequest(){
    flags.check(RobotFlag.NORMAL_ALGAE_SCORE);
  }

  public void applyHeightCapRequest(){
    flags.check(RobotFlag.APPLY_HEIGHT_CAP);
  }

  public void removeHeightCapRequest(){
    flags.check(RobotFlag.REMOVE_HEIGHT_CAP);
  }

  public void homeRequest(){
    flags.check(RobotFlag.HOMING);
  }

  public void algaeModeRequest(){
    flags.check(RobotFlag.ALGAE_MODE);
  }

  public void regularIntakeRequest(){
    flags.check(RobotFlag.NORMAL_GROUND_INTAKE);
  }

  public void intakeFailsafeRequest(){
    flags.check(RobotFlag.GROUND_INTAKE_FAILSAFE);
  }

  public void failsafeOverrideRequest(){
    flags.check(RobotFlag.OVERRIDE_STATE);
  }

  public void coralModeRequest(){
    flags.check(RobotFlag.CORAL_MODE);
  }

  public void supercycleRequest(){
    flags.check(RobotFlag.SUPERCYCLE);
  }

  public void regularCycleRequest(){
    flags.check(RobotFlag.REGULAR_CYCLE);
  }

  public void autoReefAlignRequest(){
    if(DrivetrainSubsystem.getInstance().crescendoModeEnabled){
    DrivetrainSubsystem.getInstance().setState(DrivetrainState.AUTO_REEF_ALIGN_1);
    }
  }

  public void autoAlgaeAlignRequest(){
    if(DrivetrainSubsystem.getInstance().crescendoModeEnabled){
    DrivetrainSubsystem.getInstance().setState(DrivetrainState.AUTO_ALGAE_ALIGN);
    }
  }

  public void autoCoralStationAlignRequest(){
    if(DrivetrainSubsystem.getInstance().crescendoModeEnabled){
    DrivetrainSubsystem.getInstance().setState(DrivetrainState.AUTO_CORAL_STATION_ALIGN_1);
    }
  }

  public void stopScoringRequest() {
    switch (getState()) {
      default -> setStateFromRequest(RobotState.IDLE);
    }
  }
  private static RobotManager instance;
  
  public static RobotManager getInstance() {
    if (instance == null) instance = new RobotManager(); // Make sure there is an instance (this will only run once)
    return instance;
}

}