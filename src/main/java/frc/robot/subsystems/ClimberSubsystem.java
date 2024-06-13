package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase{

    public CANSparkMax rightClimber, leftClimber;
    public Servo rightServo, leftServo;

    public ClimberSubsystem(){
        rightClimber = new CANSparkMax(ClimberConstants.rightClimberID, MotorType.kBrushed);
        leftClimber = new CANSparkMax(ClimberConstants.leftClimberID, MotorType.kBrushed);

        rightServo = new Servo(ClimberConstants.rightCLimberServoID);
        leftServo = new Servo(ClimberConstants.leftClimberServoID);

        configClimbMotors();

        //leftClimber.setControl(new Follower(rightClimber.getDeviceID(), false));
    }

    public void configClimbMotors(){
        rightClimber.restoreFactoryDefaults();
        rightClimber.setIdleMode(IdleMode.kBrake);
        rightClimber.enableVoltageCompensation(12);
        rightClimber.setSmartCurrentLimit(30);
        rightClimber.burnFlash();

        leftClimber.restoreFactoryDefaults();
        leftClimber.setIdleMode(IdleMode.kBrake);
        leftClimber.enableVoltageCompensation(12);
        leftClimber.setSmartCurrentLimit(30);
        leftClimber.burnFlash();
    }

    public void climbUp(){
        rightClimber.setVoltage(-6);
        leftClimber.setVoltage(-6);

        //open Ratchet
        rightServo.setAngle(0.0); //10
        leftServo.setAngle(0.0);
    }

    public void climbDown(){
        rightClimber.set(6);
        leftClimber.set(6);

        //setting ratchet
        rightServo.setAngle(30.0); //0
        leftServo.setAngle(30.0);
    }

    public void climbStop(){
        rightClimber.setVoltage(0.0);
        leftClimber.setVoltage(0.0);

        rightServo.setAngle(30.0); //0
        leftServo.setAngle(30.0);
    }
}
