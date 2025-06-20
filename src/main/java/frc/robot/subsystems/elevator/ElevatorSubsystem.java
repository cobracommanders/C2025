package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.RobotMode;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.StateMachine;

public class ElevatorSubsystem extends StateMachine<ElevatorState>{
  private final String name = getName();
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final CANcoder elevatorEncoder;
  private final TalonFXConfiguration left_motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ElevatorConstants.P).withKI(ElevatorConstants.I).withKD(ElevatorConstants.D).withKG(ElevatorConstants.G).withGravityType(GravityTypeValue.Elevator_Static)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((4.0 / 1.0)));
  private final TalonFXConfiguration right_motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ElevatorConstants.P).withKI(ElevatorConstants.I).withKD(ElevatorConstants.D).withKG(ElevatorConstants.G).withGravityType(GravityTypeValue.Elevator_Static)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((4.0 / 1.0)));
  private CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
  private double elevatorPosition;
  private double motorCurrent;
  private final double tolerance;
  private Follower right_motor_request = new Follower(Ports.ElevatorPorts.LMOTOR, true);
  private MotionMagicVoltage left_motor_request = new MotionMagicVoltage(0).withSlot(0);
  

  public ElevatorSubsystem() {
    super(ElevatorState.HOME_ELEVATOR);
    elevatorEncoder = new CANcoder(Ports.ElevatorPorts.ENCODER);
    right_motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    left_motor_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    left_motor_config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagicCruiseVelocity;
    left_motor_config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MotionMagicAcceleration;
    left_motor_config.MotionMagic.MotionMagicJerk = ElevatorConstants.MotionMagicJerk;
    right_motor_config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagicCruiseVelocity;
    right_motor_config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MotionMagicAcceleration;
    right_motor_config.MotionMagic.MotionMagicJerk = ElevatorConstants.MotionMagicJerk;
    leftMotor = new TalonFX(Ports.ElevatorPorts.LMOTOR);
    rightMotor = new TalonFX(Ports.ElevatorPorts.RMOTOR);
    left_motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    right_motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftMotor.getConfigurator().apply(left_motor_config);
    rightMotor.getConfigurator().apply(right_motor_config);
    canCoderConfig.MagnetSensor.MagnetOffset = Constants.ElevatorConstants.encoderOffset;
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9;
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    elevatorEncoder.getConfigurator().apply(canCoderConfig);
    tolerance = 0.1;
  }

  protected ElevatorState getNextState(ElevatorState currentState) {
    // if (getState() == ElevatorState.HOME_ELEVATOR && this.atGoal()) { 
    //   leftMotor.setPosition(0);
    //   return ElevatorState.IDLE;
    // } else {  
      return currentState;
    //}
  }

  public void setL1Row() {
    if (RobotMode.getInstance().inHighL1Mode()) {
      ElevatorPositions.L1 = ElevatorPositions.L1_ROW2;
      setElevatorPosition(ElevatorPositions.L1);
    } else {
      ElevatorPositions.L1 = ElevatorPositions.L1_ROW1;
      setElevatorPosition(ElevatorPositions.L1);
    }
  }

  public boolean atGoal() {
    return switch (getState()) {
        case IDLE -> 
          MathUtil.isNear(ElevatorPositions.IDLE, elevatorPosition, tolerance);
        case L1 ->
          MathUtil.isNear(ElevatorPositions.L1, elevatorPosition, tolerance);
        case L2 ->
          MathUtil.isNear(ElevatorPositions.L2, elevatorPosition, tolerance);
        case L3 ->
          MathUtil.isNear(ElevatorPositions.L3, elevatorPosition, tolerance);
        case LOW_ALGAE ->
          MathUtil.isNear(ElevatorPositions.LOW_ALGAE, elevatorPosition, tolerance);
        case HIGH_ALGAE ->
          MathUtil.isNear(ElevatorPositions.HIGH_ALGAE, elevatorPosition, tolerance);
        case GROUND_ALGAE ->
          MathUtil.isNear(ElevatorPositions.GROUND_ALGAE, elevatorPosition, tolerance);
        case FAILSAFE_GROUND_ALGAE ->
          MathUtil.isNear(ElevatorPositions.FAILSAFE_GROUND_ALGAE, elevatorPosition, tolerance);
        case CAPPED_L4 ->
          MathUtil.isNear(ElevatorPositions.CAPPED_L4, elevatorPosition, tolerance);
        case L4 ->
          MathUtil.isNear(ElevatorPositions.L4, elevatorPosition, tolerance);
        case L4_MAX ->
          MathUtil.isNear(ElevatorPositions.L4_MAX, elevatorPosition, tolerance);
        case CORAL_STATION ->
          MathUtil.isNear(ElevatorPositions.CORAL_STATION, elevatorPosition, tolerance);
        case HOME_ELEVATOR ->
          (motorCurrent > ElevatorConstants.homingStallCurrent);
        case INVERTED_CORAL_STATION ->
          MathUtil.isNear(ElevatorPositions.INVERTED_CORAL_STATION, elevatorPosition, tolerance);
        case PROCESSOR ->
          MathUtil.isNear(ElevatorPositions.PROCESSOR, elevatorPosition, tolerance);
      };
  }

  public void setState(ElevatorState newState) {
    setStateFromRequest(newState);
  }

  public boolean isIdle() {
    return getState() == ElevatorState.IDLE;
  }

  public void increaseSetpoint(){
    switch (getState()) {
      case L1 -> {
        ElevatorPositions.L1 += 0.1;
        setElevatorPosition(ElevatorPositions.L1);
        break;
      }
      case L2 -> {
        ElevatorPositions.L2 += 0.1;
        setElevatorPosition(ElevatorPositions.L2);
        break;
      }
      case L3 -> {
        ElevatorPositions.L3 += 0.1;
        setElevatorPosition(ElevatorPositions.L3);
        break;
      }
      case LOW_ALGAE -> {
        ElevatorPositions.LOW_ALGAE += 0.1;
        setElevatorPosition(ElevatorPositions.LOW_ALGAE);
        break;
      }
      case HIGH_ALGAE -> {
        ElevatorPositions.HIGH_ALGAE += 0.1;
        setElevatorPosition(ElevatorPositions.HIGH_ALGAE);
        break;
      }
      case GROUND_ALGAE -> {
        ElevatorPositions.GROUND_ALGAE += 0.1;
        setElevatorPosition(ElevatorPositions.GROUND_ALGAE);
        break;
      }
      case FAILSAFE_GROUND_ALGAE -> {
        ElevatorPositions.FAILSAFE_GROUND_ALGAE += 0.1;
        setElevatorPosition(ElevatorPositions.FAILSAFE_GROUND_ALGAE);
        break;
      }
      case L4_MAX -> {
        ElevatorPositions.L4_MAX += 0.1;
        setElevatorPosition(ElevatorPositions.L4_MAX);
        break;
      }
      case INVERTED_CORAL_STATION -> {
        ElevatorPositions.INVERTED_CORAL_STATION += 0.1;
        setElevatorPosition(ElevatorPositions.INVERTED_CORAL_STATION);
        break;
      }
      case PROCESSOR -> {
        ElevatorPositions.PROCESSOR += 0.1;
        setElevatorPosition(ElevatorPositions.PROCESSOR);
        break;
      }
    }
  }

  public void decreaseSetpoint(){
    switch (getState()) {
      case L1 -> {
        ElevatorPositions.L1 -= 0.1;
        setElevatorPosition(ElevatorPositions.L1);
        break;
      }
      case L2 -> {
        ElevatorPositions.L2 -= 0.1;
        setElevatorPosition(ElevatorPositions.L2);
        break;
      }
      case L3 -> {
        ElevatorPositions.L3 -= 0.1;
        setElevatorPosition(ElevatorPositions.L3);
        break;
      }
      case LOW_ALGAE -> {
        ElevatorPositions.LOW_ALGAE -= 0.1;
        setElevatorPosition(ElevatorPositions.LOW_ALGAE);
        break;
      }
      case HIGH_ALGAE -> {
        ElevatorPositions.HIGH_ALGAE -= 0.1;
        setElevatorPosition(ElevatorPositions.HIGH_ALGAE);
        break;
      }
      case GROUND_ALGAE -> {
        ElevatorPositions.GROUND_ALGAE -= 0.1;
        setElevatorPosition(ElevatorPositions.GROUND_ALGAE);
        break;
      }
      case FAILSAFE_GROUND_ALGAE -> {
        ElevatorPositions.FAILSAFE_GROUND_ALGAE -= 0.1;
        setElevatorPosition(ElevatorPositions.FAILSAFE_GROUND_ALGAE);
        break;
      }
      case L4_MAX -> {
        ElevatorPositions.L4_MAX -= 0.1;
        setElevatorPosition(ElevatorPositions.L4_MAX);
        break;
      }
      case INVERTED_CORAL_STATION -> {
        ElevatorPositions.INVERTED_CORAL_STATION -= 0.1;
        setElevatorPosition(ElevatorPositions.INVERTED_CORAL_STATION);
        break;
      }
      case PROCESSOR -> {
        ElevatorPositions.PROCESSOR -= 0.1;
        setElevatorPosition(ElevatorPositions.PROCESSOR);
        break;
      }
    }
}

  // public void syncEncoder(){
  //   leftMotor.setPosition(absolutePosition);
  // }

  @Override
  public void collectInputs(){
    //absolutePosition = elevatorEncoder.getPosition().getValueAsDouble();
    elevatorPosition = leftMotor.getPosition().getValueAsDouble();
    double leftElevatorPosition = leftMotor.getPosition().getValueAsDouble();
    double rightElevatorPosition = rightMotor.getPosition().getValueAsDouble();
    motorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    DogLog.log(name + "/Left Elevator Position", leftElevatorPosition);
    DogLog.log(name + "/Right Elevator Position", rightElevatorPosition);
    //DogLog.log(getName() + "/Elevator Current", leftMotor.getStatorCurrent().getValueAsDouble());
    //DogLog.log(getName() + "/left motor voltage", leftMotor.getMotorVoltage().getValueAsDouble());
    //DogLog.log(getName() + "/right Motor voltage", rightMotor.getMotorVoltage().getValueAsDouble());
  }

  // @Override
  // public void periodic() {
  //     super.periodic();

  //     // if (RobotManager.getInstance().getState() == RobotState.INVERTED_IDLE && RobotManager.getInstance().timeout(1) && !isSynced) {
  //     //   // syncEncoder();
  //     //   isSynced = true;
  //     // }
  //     // else if (RobotManager.getInstance().getState() != RobotState.INVERTED_IDLE) {
  //     //   isSynced = false;
  //     // }
  // }

  public void setElevatorPosition(double elevatorSetpoint){
    rightMotor.setControl(right_motor_request);
    leftMotor.setControl(left_motor_request.withPosition(elevatorSetpoint));
    DogLog.log(name + "/right Motor Setpoint", elevatorSetpoint);
  }

    @Override
    protected void afterTransition(ElevatorState newState) {
      switch (newState) {
        case IDLE -> {
          setElevatorPosition(ElevatorPositions.IDLE);
        }
        case L1 -> {
          setElevatorPosition(ElevatorPositions.L1);
        }
        case L2 -> {
          setElevatorPosition(ElevatorPositions.L2);
        }
        case L3 -> {
          setElevatorPosition(ElevatorPositions.L3);
        }
        case LOW_ALGAE -> {
          setElevatorPosition(ElevatorPositions.LOW_ALGAE);
        }
        case HIGH_ALGAE -> {
          setElevatorPosition(ElevatorPositions.HIGH_ALGAE);
        }
        case GROUND_ALGAE -> {
          setElevatorPosition(ElevatorPositions.GROUND_ALGAE);
        }
        case FAILSAFE_GROUND_ALGAE -> {
          setElevatorPosition(ElevatorPositions.FAILSAFE_GROUND_ALGAE);
        }
        case CAPPED_L4 -> {
          setElevatorPosition(ElevatorPositions.CAPPED_L4);
        }
        case L4 -> {
          setElevatorPosition(ElevatorPositions.L4);
        }
        case L4_MAX -> {
          setElevatorPosition(ElevatorPositions.L4_MAX);
        }
        case HOME_ELEVATOR -> {
          rightMotor.setControl(new VoltageOut(-0.7));
          leftMotor.setControl(new VoltageOut(-0.7));
        }
        case CORAL_STATION -> {
          setElevatorPosition(ElevatorPositions.CORAL_STATION);
        }
        case INVERTED_CORAL_STATION -> {
          setElevatorPosition(ElevatorPositions.INVERTED_CORAL_STATION);
        }
        case PROCESSOR -> {
          setElevatorPosition(ElevatorPositions.PROCESSOR);
        }
      }
    }
  
  private static ElevatorSubsystem instance;

  public static ElevatorSubsystem getInstance() {
      if (instance == null) instance = new ElevatorSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}