package frc.robot.commands;

public class RobotMode {
    public GameMode currentGameMode = GameMode.CORAL;

    public enum GameMode {CORAL, ALGAE}

    public void setCurrentGameMode(GameMode gameMode) {currentGameMode = gameMode;}
    public boolean inAlgaeMode() {return currentGameMode == GameMode.ALGAE;}
    public boolean inCoralMode() {return currentGameMode == GameMode.CORAL;}

    
    public L1Row currentL1Row = L1Row.LOWTROUGH;

    public enum L1Row {LOWTROUGH, HIGHTROUGH}

    public void setCurrentL1Mode(L1Row row) {currentL1Row = row;}
    public boolean inLowL1Mode() {return currentL1Row == L1Row.LOWTROUGH;}
    public boolean inHighL1Mode() {return currentL1Row == L1Row.HIGHTROUGH;}

    
    public CycleMode currentCycleMode = CycleMode.REGULAR_CYCLE;
    
    public enum CycleMode {SUPERCYCLE, REGULAR_CYCLE}
    
    public void setCurrentCycleMode(CycleMode cycleMode) {currentCycleMode = cycleMode;}
    public boolean inSupercycleMode() {return currentCycleMode == CycleMode.SUPERCYCLE;}
    public boolean inRegularCycleMode() {return currentCycleMode == CycleMode.REGULAR_CYCLE;}


    public IntakeMode currentIntakeMode = IntakeMode.NORMAL;
    
    public enum IntakeMode {NORMAL, FAILSAFE}
    
    public void setCurrentIntakeMode(IntakeMode intakeMode) {currentIntakeMode = intakeMode;}
    public boolean inNormalIntakeMode() {return currentIntakeMode == IntakeMode.NORMAL;}
    public boolean inFailsafeIntakeMode() {return currentIntakeMode == IntakeMode.FAILSAFE;}

    public AlgaeScoreMode currentAlgaeScoreMode = AlgaeScoreMode.REGULAR;
    
    public enum AlgaeScoreMode {REGULAR, FRONT}
    
    public void setCurrentAlgaeScoreMode(AlgaeScoreMode algaeScoreMode) {currentAlgaeScoreMode = algaeScoreMode;}
    public boolean inNormalAlgaeScoreMode() {return currentAlgaeScoreMode == AlgaeScoreMode.REGULAR;}
    public boolean inFrontAlgaeScoreMode() {return currentAlgaeScoreMode == AlgaeScoreMode.FRONT;}
    
  private static RobotMode instance;

  public static RobotMode getInstance() {
      if (instance == null) instance = new RobotMode(); // Make sure there is an instance (this will only run once)
      return instance;
  }
}

