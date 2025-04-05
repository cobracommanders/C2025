package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Ports;
import frc.robot.StateMachine;
import frc.robot.subsystems.elbow.ElbowPositions;

public class ClimberSubsystem extends StateMachine<ClimberState>{
    
  private final TalonFX lMotor;
  private final TalonFX rMotor;
  private final TalonFXConfiguration left_motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ClimberConstants.P).withKI(ClimberConstants.I).withKD(ClimberConstants.D).withKG(ClimberConstants.G).withGravityType(GravityTypeValue.Arm_Cosine)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((122.449 / 1.0)));
  private final TalonFXConfiguration right_motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ClimberConstants.P).withKI(ClimberConstants.I).withKD(ClimberConstants.D).withKG(ClimberConstants.G).withGravityType(GravityTypeValue.Arm_Cosine)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((122.449 / 1.0)));
  private ClimberState currentState;
  private double GEAR_RATIO = 122.449/1.0; // 122.449:1 gear ratio
  private double climberPosition;
  private Follower right_motor_request = new Follower(Ports.ClimberPorts.LEFT_CLIMBER_MOTOR, true);
  private MotionMagicVoltage left_motor_request = new MotionMagicVoltage(0).withSlot(0);
  
  public ClimberSubsystem() {
      super(ClimberState.IDLE);
      // motor = new LazySparkMax(Ports.IntakePorts.LMOTOR, MotorType.kBrushless);
      left_motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      right_motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      left_motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      right_motor_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      left_motor_config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagicCruiseVelocity;
      left_motor_config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.AutoMotionMagicAcceleration;
      left_motor_config.MotionMagic.MotionMagicJerk = ElevatorConstants.MotionMagicJerk;
      right_motor_config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagicCruiseVelocity;
      right_motor_config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.AutoMotionMagicAcceleration;
      right_motor_config.MotionMagic.MotionMagicJerk = ElevatorConstants.MotionMagicJerk;
      lMotor = new TalonFX(Ports.ClimberPorts.LEFT_CLIMBER_MOTOR);
      rMotor = new TalonFX(Ports.ClimberPorts.RIGHT_CLIMBER_MOTOR);  
      lMotor.getConfigurator().apply(left_motor_config);
      rMotor.getConfigurator().apply(right_motor_config);    
      
      currentState = ClimberState.IDLE;
  }

  public void setState(ClimberState newState) {
    setStateFromRequest(newState);
  }

  public boolean climberDeployed() {
    return MathUtil.isNear(ClimberPositions.DEPLOYED, climberPosition, 0.04);
  }

    @Override
    protected void afterTransition(ClimberState newState) {
      switch (newState) {
        case IDLE -> {
          lMotor.set(0.0);
          rMotor.set(0.0);
        }
        case DEEP_CLIMB_WAIT -> {
          lMotor.set(0.0);
          rMotor.set(0.0);
        }
        case DEEP_CLIMB_RETRACT -> {
          lMotor.set(-0.1);
          rMotor.set(-0.1);
        }
        case DEEP_CLIMB_DEPLOY -> {
          rMotor.setControl(right_motor_request);
          lMotor.setControl(left_motor_request.withPosition(ClimberPositions.DEPLOYED));
        }
        case DEEP_CLIMB_UNLATCH -> {
          lMotor.set(0.1);
          rMotor.set(0.1);
        }
        case DEEP_CLIMB_UNWIND -> {
          lMotor.set(0.1);
          rMotor.set(0.1);
        }
        default -> {}
      }
    }

  @Override
  public void periodic() {
    climberPosition = lMotor.getPosition().getValueAsDouble();
  }

  public boolean atGoal(){
    return true;
  }

  public void set(double speed) {
      lMotor.set(speed);
      rMotor.set(speed);
  }

  private static ClimberSubsystem instance;

  public static ClimberSubsystem getInstance() {
      if (instance == null) instance = new ClimberSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}