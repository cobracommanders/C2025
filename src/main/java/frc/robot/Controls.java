package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.RobotMode;
import frc.robot.commands.RobotMode.GameMode;
import frc.robot.drivers.Xbox;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.ElevatorPositions;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Controls {
    private  double MaxSpeed = TunerConstants.kSpeedAt12Volts; // Initial max is true top speed
    private final double TurtleSpeed = 0.1; // Reduction in speed from Max Speed, 0.1 = 10%
    private final double MaxAngularRate = Math.PI * 3.5; // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
    private final double TurtleAngularRate = Math.PI * 0.5; // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
    private double AngularRate = MaxAngularRate; // This will be updated when turtle and reset to MaxAngularRate
    public boolean isCoralMode;

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
        driver.leftTrigger().and(driver.rightBumper().negate()).onTrue(Robot.robotCommands.intakeCommand());
            driver.leftTrigger().onFalse(Robot.robotCommands.idleCommand());
        driver.rightBumper().and(driver.leftTrigger()).onTrue(Robot.robotCommands.alternateIntakeCommand());
        driver.rightTrigger().onTrue(Robot.robotCommands.scoreCommand());
            driver.rightTrigger().onFalse(Robot.robotCommands.invertIdleCommand());
        driver.leftBumper().onTrue(Robot.robotCommands.removeHeightCapCommand());
            driver.leftBumper().onFalse(Robot.robotCommands.applyHeightCapCommand());
        //driver.B().onTrue(Robot.robotCommands.autoCoralStationAlign());
        //driver.Y().onTrue(Robot.robotCommands.setDrivetrainTeleop());
        driver.B().onTrue(Robot.robotCommands.climbUnwindCommand());
        driver.B().onFalse(Robot.robotCommands.alternateIdleCommand());
        driver.Y().onTrue(Robot.robotCommands.climbRetractCommand());
        driver.Y().onFalse(Robot.robotCommands.alternateIdleCommand());
    }

    public void configureOperatorCommands(){
        operator.leftBumper().onTrue(Robot.robotCommands.invertIdleCommand());
        operator.rightBumper().onTrue(Robot.robotCommands.alternateIdleCommand());
        operator.start().and(operator.back()).onTrue(Robot.robotCommands.homeCommand());
        operator.POV180().onTrue(Robot.robotCommands.coralModeCommand());
        operator.POV0().onTrue(Robot.robotCommands.algaeModeCommand());
        operator.POV90().onTrue(runOnce(() -> ElevatorPositions.INVERTED_CORAL_STATION += 0.1));
        operator.POVMinus90().onTrue(runOnce(() -> ElevatorPositions.INVERTED_CORAL_STATION -= 0.1));
        operator.Y().onTrue(Robot.robotCommands.LowReefCommand());
        operator.B().onTrue(Robot.robotCommands.HighReefCommand());
        operator.X().onTrue(Robot.robotCommands.L2Command());
        operator.A().onTrue(Robot.robotCommands.L1Command());
        operator.leftTrigger().and(operator.rightTrigger()).onTrue(Robot.robotCommands.climbCommand());
    }

    private static Controls instance;

    public static Controls getInstance() {
      if (instance == null) instance = new Controls(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}
