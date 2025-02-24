package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.drivers.Xbox;
import frc.robot.subsystems.climber.ClimberState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Controls {
    private  double MaxSpeed = TunerConstants.kSpeedAt12Volts; // Initial max is true top speed
    private final double TurtleSpeed = 0.1; // Reduction in speed from Max Speed, 0.1 = 10%
    private final double MaxAngularRate = Math.PI * 3.5; // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
    private final double TurtleAngularRate = Math.PI * 0.5; // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
    private double AngularRate = MaxAngularRate; // This will be updated when turtle and reset to MaxAngularRate

     SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withDeadband(MaxSpeed * 0.1) // Deadband is handled on input
      .withRotationalDeadband(AngularRate * 0.1);

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
        driver.leftTrigger().and(driver.rightBumper().negate()).onTrue(Robot.robotCommands.invertedIntakeCommand());
            driver.leftTrigger().onFalse(Robot.robotCommands.invertIdleCommand());
        driver.rightBumper().and(driver.leftTrigger()).onTrue(Robot.robotCommands.intakeCommand());
        driver.rightTrigger().onTrue(Robot.robotCommands.scoreCommand());
            driver.rightTrigger().onFalse(Robot.robotCommands.invertIdleCommand());
        driver.leftBumper().onTrue(Robot.robotCommands.removeHeightCapCommand());
            driver.leftBumper().onFalse(Robot.robotCommands.applyHeightCapCommand());
        driver.B().onTrue(Robot.robotCommands.autoCoralStationAlign());
        driver.Y().onTrue(Robot.robotCommands.setDrivetrainTeleop());
        // driver.B().onTrue(runOnce(()-> ClimberSubsystem.getInstance().setState(ClimberState.DEEP_CLIMB_RETRACT)));
        // driver.B().onFalse(runOnce(()-> ClimberSubsystem.getInstance().setState(ClimberState.IDLE)));
        // driver.Y().onTrue(runOnce(()-> ClimberSubsystem.getInstance().setState(ClimberState.DEEP_CLIMB_DEPLOY)));
        // driver.Y().onFalse(runOnce(()-> ClimberSubsystem.getInstance().setState(ClimberState.IDLE)));
    }

    public void configureOperatorCommands(){
        operator.leftBumper().onTrue(Robot.robotCommands.invertIdleCommand());
        operator.rightBumper().onTrue(Robot.robotCommands.idleCommand());
        operator.leftTrigger().onTrue(Commands.runOnce(()-> ElevatorSubsystem.getInstance().setState(ElevatorState.HOME_ELEVATOR)));
        operator.start().and(operator.back()).onTrue(Robot.robotCommands.homeCommand());
        operator.Y().onTrue(Robot.robotCommands.L3Command());
        operator.B().onTrue(Robot.robotCommands.L4Command());
        operator.X().onTrue(Robot.robotCommands.L2Command());
        operator.A().onTrue(Robot.robotCommands.L1Command());
        operator.leftTrigger().and(operator.rightTrigger()).onTrue(Robot.robotCommands.climbCommand());
        operator.POV0().onTrue(runOnce(() -> KickerSubsystem.getInstance().disabled = true));
        operator.POV180().onTrue(runOnce(() -> KickerSubsystem.getInstance().disabled = false));
    }
}
