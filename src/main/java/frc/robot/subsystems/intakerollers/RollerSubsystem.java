package frc.robot.subsystems.intakerollers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.StateMachine;



public class RollerSubsystem extends StateMachine<RollerState>{
    public final TalonFX rollerMotor;
    private final TalonFXConfiguration motor_config = new TalonFXConfiguration();
    private Timer intakeTimer = new Timer();
    private double rollerSpeed;
    private double rollerStatorCurrent;
    
    public RollerSubsystem() {
      super(RollerState.IDLE);
      rollerMotor = new TalonFX(Ports.IntakeRollerPorts.INTAKE_ROLLER_MOTOR);
      motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      motor_config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.8;
      motor_config.CurrentLimits.StatorCurrentLimit = 100;
      rollerMotor.getConfigurator().apply(motor_config);
    }

    protected RollerState getNextState(RollerState currentState) {
      return currentState;
    }

    @Override
    public void collectInputs(){
      rollerSpeed = rollerMotor.get();
      rollerStatorCurrent = rollerMotor.getStatorCurrent().getValueAsDouble();
      DogLog.log(getName() + "/Intake Roller Motor Stator Current", rollerStatorCurrent);
    }
  
    public void setState(RollerState newState) {
        setStateFromRequest(newState);
    }
  
    public void setRollerPositions(double rollerSpeed){
      DogLog.log(getName() + "/Roller speed", rollerSpeed);
      rollerMotor.set(rollerSpeed);
    }
  
      @Override
      protected void afterTransition(RollerState newState) {
        switch (newState) {
          case IDLE -> {
            setRollerPositions(RollerSpeeds.IDLE);
          }
          case INTAKE -> {
            setRollerPositions(RollerSpeeds.INTAKE);
          }
          case OUTTAKE -> {
            setRollerPositions(RollerSpeeds.OUTTAKE);
          }
          case PROCESSOR -> {
            setRollerPositions(RollerSpeeds.PROCESSOR);
          }
          default -> {}
        }
      }
  
    private static RollerSubsystem instance;
  
    public static RollerSubsystem getInstance() {
        if (instance == null) instance = new RollerSubsystem(); // Make sure there is an instance (this will only run once)
        return instance;
    }
  }