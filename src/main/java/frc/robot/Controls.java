package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.drivers.Xbox;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSpeeds;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Controls {

     
    public final Xbox driver = new Xbox(OIConstants.DRIVER_CONTROLLER_ID);
    public final Xbox operator = new Xbox(OIConstants.OPERATOR_CONTROLLER_ID);

    public Controls() {
        driver.setTriggerThreshold(0.2);
        driver.setDeadzone(0.15);
        operator.setTriggerThreshold(0.2);
        operator.setDeadzone(0.2);
    }

    public void configureDriverCommands() {
        driver.A().onTrue(runOnce(() -> CommandSwerveDrivetrain.getInstance().setYaw(Robot.alliance.get())));
        driver.leftTrigger().onTrue(Robot.robotCommands.intakeCommand());
            driver.leftTrigger().onFalse(Robot.robotCommands.intakeIdleCommand());
        driver.rightBumper().onTrue(Robot.robotCommands.autoAlignCommand());
        driver.rightTrigger().onTrue(Robot.robotCommands.scoreCommand());
            driver.rightTrigger().onFalse(Robot.robotCommands.scoreIdleCommand());
        driver.leftBumper().onTrue(Robot.robotCommands.removeHeightCapCommand());
            driver.leftBumper().onFalse(Robot.robotCommands.applyHeightCapCommand());
        driver.start().onTrue(Robot.robotCommands.IntakeDisableCommand());
        driver.back().onTrue(Robot.robotCommands.IntakeEnableCommand());
        // driver.B().onTrue(Robot.robotCommands.autoCoralStationAlign());
        // driver.X().onTrue(Robot.robotCommands.autoReefAlign());
        // driver.Y().onTrue(Robot.robotCommands.setDrivetrainTeleop());
        // driver.X().onTrue(runOnce(() -> ClimberSubsystem.getInstance().setState(ClimberState.PID_DEEP_CLIMB_RETRACT)).andThen(runOnce(() -> ClimberWheelSubsystem.getInstance().setState(ClimberWheelState.INTAKE_CAGE))));
        // driver.X().onFalse(runOnce(() -> ClimberSubsystem.getInstance().setState(ClimberState.IDLE)).andThen(runOnce(() -> ClimberWheelSubsystem.getInstance().setState(ClimberWheelState.IDLE))));

        // driver.Y().onTrue(runOnce(() -> ClimberSubsystem.getInstance().setState(ClimberState.MANUAL_DEEP_CLIMB_UNWIND)).andThen(runOnce(() -> ClimberWheelSubsystem.getInstance().setState(ClimberWheelState.INTAKE_CAGE))));
        // driver.Y().onFalse(runOnce(() -> ClimberSubsystem.getInstance().setState(ClimberState.IDLE)).andThen(runOnce(() -> ClimberWheelSubsystem.getInstance().setState(ClimberWheelState.IDLE))));
        // driver.B().onTrue(runOnce(() -> ClimberSubsystem.getInstance().setState(ClimberState.MANUAL_DEEP_CLIMB_RETRACT)));
        // driver.B().onFalse(runOnce(() -> ClimberSubsystem.getInstance().setState(ClimberState.IDLE)).andThen(runOnce(() -> ClimberWheelSubsystem.getInstance().setState(ClimberWheelState.IDLE))));
        
        driver.B().onTrue(Robot.robotCommands.climbRetractCommand());
        driver.Y().onTrue(Robot.robotCommands.failsafeClimbRetractCommand());
        driver.X().onTrue(Robot.robotCommands.algaeScoreSwapCommand());
        // driver.Y().onTrue(Robot.robotCommands.climbUnwindCommand());
        driver.POV0().onTrue(runOnce(() -> ElevatorSubsystem.getInstance().increaseSetpoint()));
        driver.POV180().onTrue(runOnce(() -> ElevatorSubsystem.getInstance().decreaseSetpoint()));
        driver.POV90().onTrue(runOnce(()-> DrivetrainSubsystem.getInstance().crescendoModeEnabled = true));
        driver.POVMinus90().onTrue(runOnce(()-> DrivetrainSubsystem.getInstance().crescendoModeEnabled = false));
        
    }

    public void configureOperatorCommands(){
        operator.leftBumper().onTrue(Robot.robotCommands.idleCommand());
        operator.rightBumper().onTrue(Robot.robotCommands.algaeIdleCommand());
        operator.start().and(operator.back()).onTrue(Robot.robotCommands.homeCommand());
        operator.back().onTrue(runOnce(() -> toggleFunnel()));
        operator.POV180().onTrue(Robot.robotCommands.coralModeCommand());
        operator.POVMinus90().onTrue(Robot.robotCommands.L1ToggleCommand());
        operator.POV0().onTrue(Robot.robotCommands.algaeModeCommand());
        operator.POV90().onTrue(Robot.robotCommands.cycleModeCommand());
        operator.Y().onTrue(Robot.robotCommands.LowReefCommand());
        operator.B().onTrue(Robot.robotCommands.HighReefCommand());
        operator.X().onTrue(Robot.robotCommands.L2MultiCommand());
        operator.A().onTrue(Robot.robotCommands.ProcessorCommand());
        operator.leftTrigger().and(operator.rightTrigger()).onTrue(Robot.robotCommands.climbCommand());
        operator.rightStick().onTrue(Robot.robotCommands.failsafeClimbCommand());
    }
    //add: comment in IntakeRollerSubsystem
    public void configureTestControls(){
        driver.A().onTrue(runOnce(() -> ManipulatorSubsystem.getInstance().setManipulatorSpeeds(ManipulatorSpeeds.INTAKE_CORAL)));
        driver.A().onFalse(runOnce(() -> ManipulatorSubsystem.getInstance().setManipulatorSpeeds(0)));
        driver.rightBumper().onTrue(runOnce(() -> ElbowSubsystem.getInstance().tickUp()));
        driver.leftBumper().onTrue(runOnce(() -> ElbowSubsystem.getInstance().tickDown()));

        driver.B().onTrue(runOnce(() -> WristSubsystem.getInstance().tickUp()));
        driver.X().onTrue(runOnce(() -> WristSubsystem.getInstance().tickDown()));

        driver.POV90().onTrue(runOnce(() -> ElevatorSubsystem.getInstance().increaseSetpoint()));
        driver.POV180().onTrue(runOnce(() -> ElevatorSubsystem.getInstance().decreaseSetpoint()));
    }

    boolean isFunnel = false;
    public void toggleFunnel(){
        WristSubsystem.getInstance().toggleFunnel();
        ElbowSubsystem.getInstance().toggleFunnel();
        ElevatorSubsystem.getInstance().toggleFunnel();
        isFunnel = !isFunnel;
        if(isFunnel){
            operator.rumble(1);
        }else{
            operator.rumble(0);
        }
    }


    private static Controls instance;

    public static Controls getInstance() {
      if (instance == null) instance = new Controls(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
