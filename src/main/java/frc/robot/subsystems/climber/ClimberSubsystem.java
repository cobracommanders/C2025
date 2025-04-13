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

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Ports;
import frc.robot.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState>{
    
  private final TalonFX lMotor;
  private final TalonFX rMotor;
  private TalonFXConfiguration left_motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ClimberConstants.P).withKI(ClimberConstants.I).withKD(ClimberConstants.D).withKG(ClimberConstants.G).withGravityType(GravityTypeValue.Arm_Cosine)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((122.449 / 1.0)));
  private TalonFXConfiguration right_motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(ClimberConstants.P).withKI(ClimberConstants.I).withKD(ClimberConstants.D).withKG(ClimberConstants.G).withGravityType(GravityTypeValue.Arm_Cosine)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((122.449 / 1.0)));
  private double climberPosition;
  private Follower right_motor_request = new Follower(Ports.ClimberPorts.LEFT_CLIMBER_MOTOR, true);
  private MotionMagicVoltage left_motor_request = new MotionMagicVoltage(0).withSlot(0);
  
  public ClimberSubsystem() {
      super(ClimberState.IDLE);
      left_motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      right_motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      left_motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      right_motor_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      left_motor_config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.DeployMotionMagicCruiseVelocity;
      left_motor_config.MotionMagic.MotionMagicAcceleration = ClimberConstants.DeployMotionMagicAcceleration;
      left_motor_config.MotionMagic.MotionMagicJerk = ClimberConstants.MotionMagicJerk;
      right_motor_config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.DeployMotionMagicCruiseVelocity;
      right_motor_config.MotionMagic.MotionMagicAcceleration = ClimberConstants.DeployMotionMagicAcceleration;
      right_motor_config.MotionMagic.MotionMagicJerk = ClimberConstants.MotionMagicJerk;
      lMotor = new TalonFX(Ports.ClimberPorts.LEFT_CLIMBER_MOTOR);
      rMotor = new TalonFX(Ports.ClimberPorts.RIGHT_CLIMBER_MOTOR);  
      lMotor.getConfigurator().apply(left_motor_config);
      rMotor.getConfigurator().apply(right_motor_config);
  }

  public void setState(ClimberState newState) {
    setStateFromRequest(newState);
  }

  public boolean climberDeployed() {
    return MathUtil.isNear(ClimberPositions.DEPLOYED, climberPosition, 0.04);
  }

  // public void setDeployConfig() {
  //   left_motor_config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.DeployMotionMagicCruiseVelocity;
  //   left_motor_config.MotionMagic.MotionMagicAcceleration = ClimberConstants.DeployMotionMagicAcceleration;
  //   right_motor_config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.DeployMotionMagicCruiseVelocity;
  //   right_motor_config.MotionMagic.MotionMagicAcceleration = ClimberConstants.DeployMotionMagicAcceleration;
  //   lMotor.getConfigurator().apply(left_motor_config);
  //   rMotor.getConfigurator().apply(right_motor_config);
  // }
  public void setRetractConfig() {
    left_motor_config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.RetractMotionMagicCruiseVelocity;
    left_motor_config.MotionMagic.MotionMagicAcceleration = ClimberConstants.RetractMotionMagicAcceleration;
    right_motor_config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.RetractMotionMagicCruiseVelocity;
    right_motor_config.MotionMagic.MotionMagicAcceleration = ClimberConstants.RetractMotionMagicAcceleration;
    lMotor.getConfigurator().apply(left_motor_config);
    rMotor.getConfigurator().apply(right_motor_config);
  }

    @Override
    protected void afterTransition(ClimberState newState) {
      switch (newState) {
        case IDLE -> {
          set(0.0);
        }
        case DEEP_CLIMB_WAIT -> {
          setClimberPosition(ClimberPositions.DEPLOYED);
          
        }
        case DEEP_CLIMB_RETRACT -> {
          setClimberPosition(ClimberPositions.MAX_EXTENSION);
        }
        case DEEP_CLIMB_DEPLOY -> {
          setClimberPosition(ClimberPositions.DEPLOYED);
        }
        case DEEP_CLIMB_UNLATCH -> {
          set(0.1);
        }
        case MANUAL_DEEP_CLIMB_RETRACT -> {
          set(0.1);
        }
        case MANUAL_DEEP_CLIMB_UNWIND -> {
          set(-0.1);
        }
        case DEEP_CLIMB_UNWIND -> {
          setClimberPosition(ClimberPositions.DEPLOYED);
        }
        default -> {}
      }
    }

  @Override
  public void periodic() {
    climberPosition = lMotor.getPosition().getValueAsDouble();
    DogLog.log(getName() + "/Climber Position", climberPosition);
  }

  public boolean atGoal(){
    return true;
  }

  public void set(double speed) {
    lMotor.set(speed);
    rMotor.set(speed);
  }

  public void setClimberPosition(double climberSetpoint){
    rMotor.setControl(right_motor_request);
    lMotor.setControl(left_motor_request.withPosition(climberSetpoint));
    DogLog.log(getName() + "/Left motor setpoint", climberSetpoint);
  }

  private static ClimberSubsystem instance;

  public static ClimberSubsystem getInstance() {
      if (instance == null) instance = new ClimberSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}