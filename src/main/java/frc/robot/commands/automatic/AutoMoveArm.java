package frc.robot.commands.automatic;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ScoringSubsystem;

public class AutoMoveArm extends Command {
    private ScoringSubsystem arm;
    private double setpoint;
    private boolean end;
    private TrapezoidProfile.State state;

    public AutoMoveArm(ScoringSubsystem arm, double setpoint){
        addRequirements(arm);
        this.arm = arm;
        this.setpoint = setpoint;
        end = false;
    }

    @Override
    public void initialize(){
        end = false;
    }

    @Override
    public void execute(){
        if(arm.isInTolarance(arm.getPos(), setpoint, ArmConstants.kTolerance)){
            end = true;
        } else {
            state = new State(setpoint, 0);
            arm.runToPosition(state);
            end = false;
        }
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return end;
    }
}