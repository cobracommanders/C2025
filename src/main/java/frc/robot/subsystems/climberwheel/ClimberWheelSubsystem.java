package frc.robot.subsystems.climberwheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.StateMachine;


public class ClimberWheelSubsystem extends StateMachine<ClimberWheelState>{
    private final String name = getName();
    public final TalonFX climberWheelMotor;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration();
    private double climberWheelSpeed;
    private double climberWheelStatorCurrent;
    private boolean hasCage = false;
    
    public ClimberWheelSubsystem() {
        super(ClimberWheelState.IDLE);
        climberWheelMotor = new TalonFX(Ports.ClimberWheelPorts.CLIMBER_WHEEL_MOTOR);
        motor_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor_config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.8;
        motor_config.CurrentLimits.StatorCurrentLimit = 60;
        climberWheelMotor.getConfigurator().apply(motor_config);
      }

    protected ClimberWheelState getNextState(ClimberWheelState currentState) {
      return currentState;
    }

    public void setState(ClimberWheelState newState) {
        setStateFromRequest(newState);
    }

    @Override
    public void collectInputs(){
      //climberWheelSpeed = climberWheelMotor.get();
      climberWheelStatorCurrent = climberWheelMotor.getStatorCurrent().getValueAsDouble();
      DogLog.log(name + "/Climber wheel motor stator Current", climberWheelStatorCurrent);
      //DogLog.log(getName() + "/Climber wheel motor speed", climberWheelSpeed);
    }

    public boolean hasCage(){
      if (climberWheelStatorCurrent > Constants.ClimberWheelConstants.cageStallCurrent){
        hasCage = true;
        return true;
      } else {
        hasCage = false;
        return false;
      } 
    }

      public void setClimberWheelPositions(double climberWheelSpeed) {
        DogLog.log(name + "/Climber wheel speed", climberWheelSpeed);
        climberWheelMotor.set(climberWheelSpeed);
      }
      
      @Override
      protected void afterTransition(ClimberWheelState newState) {
        switch (newState) {
            case IDLE -> {
              setClimberWheelPositions(ClimberWheelSpeeds.IDLE);
            }
            case INTAKE_CAGE -> {
              setClimberWheelPositions(ClimberWheelSpeeds.INTAKE_CAGE);
            }
            default -> {}
        }
    }
    private static ClimberWheelSubsystem instance;
  
    public static ClimberWheelSubsystem getInstance() {
        if (instance == null) instance = new ClimberWheelSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
    }
