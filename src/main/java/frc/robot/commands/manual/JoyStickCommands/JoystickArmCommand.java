package frc.robot.commands.manual.JoyStickCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ScoringSubsystem;

public class JoystickArmCommand extends Command{
    private ScoringSubsystem subsystem;
    private ClimberSubsystem climbSub;
    private Supplier<Boolean> intakeButtonForward, armUp, armDown, climbUp, climbDown;

    public JoystickArmCommand(ScoringSubsystem subsystem, ClimberSubsystem climbSub,
     Supplier<Boolean> intakeButtonForward,
     Supplier<Boolean> armUp, Supplier<Boolean> armDown,
     Supplier<Boolean> climbUp, Supplier<Boolean> climbDown){
        addRequirements(subsystem, climbSub);
        this.climbSub = climbSub;
        this.subsystem = subsystem;

        this.climbUp = climbUp;
        this.climbDown = climbDown;

        this.intakeButtonForward = intakeButtonForward;

        this.armUp = armUp;
        this.armDown = armDown;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(intakeButtonForward.get()){
        subsystem.intakeFoward();
        subsystem.shooter(-.125);
    } else {
        subsystem.intakeOff();
        subsystem.shooter(0.0);
    }

    if(armUp.get()){
      subsystem.moveArm(.3);
    }
    if(armDown.get()){
      subsystem.moveArm(-.375);
    }
    if(armUp.get() == false && armDown.get() == false){
      subsystem.moveArm(0);
    }

    if(climbUp.get()){
      climbSub.climbUp();;
    }
    if(climbDown.get()){
      climbSub.climbDown();;
    }
    if(climbUp.get() == false && climbDown.get() == false){
      climbSub.climbStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
