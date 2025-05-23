package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Set;

import dev.doglog.DogLog;

/** A state machine backed by {@link LifecycleSubsystem}. */
public abstract class StateMachine<S extends Enum<S>> extends SubsystemBase {
  private S state;
  private boolean isInitialized = false;
  private double lastTransitionTimestamp = Timer.getFPGATimestamp();

  /**
   * Creates a new state machine.
   *
   * @param priority The subsystem priority of this subsystem in {@link LifecycleSubsystemManager}.
   * @param initialState The initial/default state of the state machine.
   */
  protected StateMachine(S initialState) {
    state = initialState;
  }

  /** Processes collecting inputs, state transitions, and state actions. */
  @Override
  public void periodic() {
    if (!isInitialized) {
      doTransition();
      isInitialized = true;
    }

    collectInputs();

    setStateFromRequest(getNextState(state));
  }

  /**
   * Gets the current state.
   *
   * @return The current state.
   */
  public S getState() {
    return state;
  }

  /**
   * Creates a command that waits until this state machine is in the given state.
   *
   * @param goalState The state to wait for.
   * @return A command that waits until the state is equal to the goal state.
   */
  public Command waitForState(S goalState) {
    return Commands.waitUntil(() -> this.state == goalState);
  }

  /**
   * Creates a command that waits until this state machine is in any of the given states.
   *
   * @param goalStates A set of the states to wait for.
   * @return A command that waits until the state is equal to any of the goal states.
   */
  public Command waitForStates(Set<S> goalStates) {
    return Commands.waitUntil(() -> goalStates.contains(this.state));
  }

  /**
   * Called each loop before processing transitions. Used for retrieving sensor values, etc.
   *
   * <p>Default behavior is to do nothing.
   */
  protected void collectInputs() {}

  /**
   * Process transitions from one state to another.
   *
   * <p>Default behavior is to stay in the current state indefinitely.
   *
   * @param currentState The current state.
   * @return The new state after processing transitions.
   */
  protected S getNextState(S currentState) {
    return currentState;
  }

  /**
   * Runs once after entering a new state. This is where you should run state actions.
   *
   * @param newState The newly entered state.
   */
  protected void afterTransition(S newState) {}

  /**
   * Used to change to a new state when a request is made. Will also trigger all logic that should
   * happen when a state transition occurs.
   *
   * @param requestedState The new state to transition to.
   */
  protected void setStateFromRequest(S requestedState) {
    if (state == requestedState) {
      // No change
      return;
    }

    state = requestedState;
    doTransition();
  }

  /**
   * Checks if the current state has been in for longer than the given duration. Used for having
   * timeout logic in state transitions.
   *
   * @param duration The timeout duration (in seconds) to use.
   * @return Whether the current state has been active for longer than the given duration.
   */
  public boolean timeout(double duration) {
    var currentStateDuration = Timer.getFPGATimestamp() - lastTransitionTimestamp;

    return currentStateDuration > duration;
  } 

  /** Run side effects that occur when a state transition happens. */
  private void doTransition() {


    lastTransitionTimestamp = Timer.getFPGATimestamp();
    DogLog.log(this.getName() + "/State", state);

    afterTransition(state);
  }
}