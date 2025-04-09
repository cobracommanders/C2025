package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports;
import frc.robot.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState>{
    
  private final TalonFX intakeMotor;
  private final TalonFXConfiguration motor_config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(IntakeConstants.P).withKI(IntakeConstants.I).withKD(IntakeConstants.D).withKG(IntakeConstants.G).withGravityType(GravityTypeValue.Arm_Cosine)).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio((8.5714 / 1.0)));
  private double intakePosition;
  private final double tolerance;
  private boolean brakeModeEnabled;
  private double motorCurrent;

  private MotionMagicVoltage motor_request = new MotionMagicVoltage(0).withSlot(0);
  
  public IntakeSubsystem() {
    super(IntakeState.IDLE);
    motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeMotor = new TalonFX(Ports.IntakePorts.INTAKE_MOTOR);
    intakeMotor.getConfigurator().apply(motor_config);
    motor_config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.MotionMagicCruiseVelocity;
    motor_config.MotionMagic.MotionMagicAcceleration = IntakeConstants.MotionMagicAcceleration;
    motor_config.MotionMagic.MotionMagicJerk = IntakeConstants.MotionMagicJerk;
    tolerance = 0.04;
    brakeModeEnabled = false;
  }
  
//   protected IntakeState getNextState(IntakeState currentState) {
//     if (getState() == IntakeState.HOME_ELBOW && this.atGoal()) { 
//       motor.setPosition(0);
//       return ElbowState.INVERTED_IDLE;
//     } else {
//       return currentState;
//   }

   public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> 
        MathUtil.isNear(IntakePositions.IDLE, intakePosition, tolerance);
      case INTAKE -> 
        MathUtil.isNear(IntakePositions.INTAKE, intakePosition, tolerance);
      case OUTTAKE ->
        MathUtil.isNear(IntakePositions.OUTTAKE, intakePosition, tolerance);
      case PROCESSOR ->
        MathUtil.isNear(IntakePositions.PROCESSOR, intakePosition, tolerance);
      case CAGE_FLIP ->
        MathUtil.isNear(IntakePositions.CAGE_FLIP, intakePosition, tolerance);
     };
  }

  public void setState(IntakeState newState) {
      setStateFromRequest(newState);
  }

  @Override
  public void collectInputs() {
    intakePosition = intakeMotor.getPosition().getValueAsDouble();
    motorCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();
    DogLog.log(getName() + "/Intake Position", intakePosition);
    DogLog.log(getName() + "/Intake current", motorCurrent);
    DogLog.log(getName() + "/Intake AtGoal", atGoal());
  }

  @Override
  public void periodic() {
    super.periodic();

    if (DriverStation.isDisabled() && brakeModeEnabled == true) {
      motor_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      intakeMotor.getConfigurator().apply(motor_config);
      brakeModeEnabled = false;
    }
    else if (DriverStation.isEnabled() && brakeModeEnabled == false)  {
      motor_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      intakeMotor.getConfigurator().apply(motor_config);
      brakeModeEnabled = true;
    }
  }

  public void setIntakePosition(double position) {
    intakeMotor.setControl(motor_request.withPosition(position));
    DogLog.log(getName() + "/Elbow Setpoint", position);
  }

    @Override
    protected void afterTransition(IntakeState newState) {
      switch (newState) {
        case IDLE -> {
          setIntakePosition(IntakePositions.IDLE);
        }
        case INTAKE -> {
          setIntakePosition(IntakePositions.INTAKE);
        }
        case OUTTAKE -> {
          setIntakePosition(IntakePositions.OUTTAKE);
        }
        case PROCESSOR -> {
          setIntakePosition(IntakePositions.PROCESSOR);
        }
        case CAGE_FLIP -> {
          setIntakePosition(IntakePositions.CAGE_FLIP);
        }
      }
    }

  private static IntakeSubsystem instance;

  public static IntakeSubsystem getInstance() {
      if (instance == null) instance = new IntakeSubsystem(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}