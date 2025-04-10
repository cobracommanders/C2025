package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.StateMachine;



public class ManipulatorSubsystem extends StateMachine<ManipulatorState>{
    public final TalonFX manipulatorMotor;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration();
    private double manipulatorStatorCurrent;
    
    public ManipulatorSubsystem() {
      super(ManipulatorState.IDLE);
      manipulatorMotor = new TalonFX(Ports.ManipulatorPorts.MANIPULATOR_MOTOR);
      motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      motor_config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.8;
      motor_config.CurrentLimits.StatorCurrentLimit = 90;
      manipulatorMotor.getConfigurator().apply(motor_config);
    }

    protected ManipulatorState getNextState(ManipulatorState currentState) {
      return currentState;
    }

    @Override
    public void collectInputs(){
      manipulatorStatorCurrent = manipulatorMotor.getStatorCurrent().getValueAsDouble();
      DogLog.log(getName() + "/Motor Stator Current", manipulatorStatorCurrent);
    }
  
    public void setState(ManipulatorState newState) {
        setStateFromRequest(newState);
    }

    public boolean hasCoral(){
      if (manipulatorStatorCurrent > Constants.ManipulatorConstants.coralStallCurrent){
        return true;
      } else {
        return false;
      }
    }

    public boolean hasAlgae(){
      if (manipulatorStatorCurrent > Constants.ManipulatorConstants.coralStallCurrent){
        return true;
      } else {
        return false;
      }
    }
  
    public void setManipulatorSpeeds(double manipulatorSpeed){
      DogLog.log(getName() + "/Manipulator speed", manipulatorSpeed);
      manipulatorMotor.set(manipulatorSpeed);
    }
  
      @Override
      protected void afterTransition(ManipulatorState newState) {
        switch (newState) {
          case IDLE -> {
            setManipulatorSpeeds(ManipulatorSpeeds.IDLE);
          }
          case INTAKE_CORAL -> {
            setManipulatorSpeeds(ManipulatorSpeeds.INTAKE_CORAL);
          }
          case INTAKE_GROUND_ALGAE -> {
            setManipulatorSpeeds(ManipulatorSpeeds.INTAKE_GROUND_ALGAE);
          }
          case INTAKE_GROUND_ALGAE_FAILSAFE -> {
            setManipulatorSpeeds(ManipulatorSpeeds.FAILSAFE_GROUND_ALGAE);
          }
          case AFTER_INTAKE -> {
            setManipulatorSpeeds(ManipulatorSpeeds.AFTER_INTAKE);
          }
          case L1 -> {
            setManipulatorSpeeds(ManipulatorSpeeds.L1);
          }
          case L2 -> {
            setManipulatorSpeeds(ManipulatorSpeeds.L2);
          }
          case L3 -> {
            setManipulatorSpeeds(ManipulatorSpeeds.L3);
          }
          case L4 -> {
            setManipulatorSpeeds(ManipulatorSpeeds.L4);
          }
          case PRE_SCORE -> {
            setManipulatorSpeeds(ManipulatorSpeeds.PRE_SCORE);
          }
          case INTAKE_ALGAE -> {
            setManipulatorSpeeds(ManipulatorSpeeds.INTAKE_ALGAE);
          }
          case SCORE_ALGAE -> {
            setManipulatorSpeeds(ManipulatorSpeeds.SCORE_ALGAE);
          }
          case OUTTAKE_ALGAE -> {
            setManipulatorSpeeds(ManipulatorSpeeds.OUTTAKE_ALGAE);
          }
          case SCORE_PROCESSOR -> {
            setManipulatorSpeeds(ManipulatorSpeeds.SCORE_PROCESSOR);
          }
          default -> {}
        }
      }
  
    private static ManipulatorSubsystem instance;
  
    public static ManipulatorSubsystem getInstance() {
        if (instance == null) instance = new ManipulatorSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
  }