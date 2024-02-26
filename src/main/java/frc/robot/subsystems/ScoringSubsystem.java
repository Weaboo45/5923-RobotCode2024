package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import frc.lib.util.CANCoderUtil;
//import frc.lib.util.CANSparkMaxUtil;
//import frc.lib.util.CANCoderUtil.CCUsage;
//import frc.lib.util.CANSparkMaxUtil.Usage;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringSubsystem extends SubsystemBase {
    //arm motors/encoders
    private CANSparkMax leftArmMotor;
    private CANSparkMax rightArmMotor;
    private RelativeEncoder leftArmEncoder;
    private RelativeEncoder rightArmEncoder;

    //shooter motors
    private CANSparkMax topShooterMotor;
    private CANSparkMax bottomShooterMotor;

    //intake motor
    private CANSparkMax intakeMotor;

    private SparkPIDController armController;
    private Encoder amrThroughBoreEncoder;

    public ScoringSubsystem() {

        //absolute encoder
        //amrThroughBoreEncoder = new AbsoluteEncoder();
    
        //arm motors
        leftArmMotor = new CANSparkMax(Constants.leftArmMotorID, MotorType.kBrushless);
        rightArmMotor = new CANSparkMax(Constants.rightArmMotorID, MotorType.kBrushless);

        //arm encoders
        //leftArmEncoder = leftArmMotor.getEncoder();
        //rightArmEncoder = rightArmMotor.getEncoder();
        
        //shooter motors
        topShooterMotor = new CANSparkMax(Constants.topShooterMotorID, MotorType.kBrushless);
        bottomShooterMotor = new CANSparkMax(Constants.bottomShooterMotorID, MotorType.kBrushless);
        
        //intake motor
        intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushed);
        //angleMotorID = new CANSparkmax(Constants.angleMotorID, MotorType.kBrushless)

        //motor followers
        //leftArmMotor.follow(rightArmMotor, false);
        //bottomShooterMotor.follow(topShooterMotor, true);

        //PID controller
        //armController = rightArmMotor.getPIDController();

        //motor configs
        configLeftArmMotor();
        configRightArmMotor();
        configBottorShooterMotor();
        configTopShooterMotor();
        configIntakeMotor();
  }

  private void configRightArmMotor() {
    rightArmMotor.restoreFactoryDefaults();
    //CANSparkMaxUtil.setCANSparkMaxBusUsage(rightArmMotor, Usage.kPositionOnly);
    rightArmMotor.setSmartCurrentLimit(40);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    //rightArmEncoder.setPositionConversionFactor(Constants.TURN_MOTOR_PCONVERSION);
    /*
    armController.setPositionPIDWrappingEnabled(true);
    armController.setPositionPIDWrappingMinInput(-180.0);
    armController.setPositionPIDWrappingMaxInput(180.0);
    armController.setP(Constants.ROTATE_KP);
    armController.setI(Constants.ROTATE_KI);
    armController.setD(Constants.ROTATE_KD);
    armController.setFF(0.0);
    */
    rightArmMotor.enableVoltageCompensation(12);
    rightArmMotor.burnFlash();
    //resetToAbsolute();
  }

  private void configLeftArmMotor() {
    leftArmMotor.restoreFactoryDefaults();
    //CANSparkMaxUtil.setCANSparkMaxBusUsage(leftArmMotor, Usage.kPositionOnly);
    leftArmMotor.setSmartCurrentLimit(40);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    //leftArmEncoder.setVelocityConversionFactor(Constants.DRIVE_MOTOR_VCONVERSION);
    //leftArmEncoder.setPositionConversionFactor(Constants.DRIVE_MOTOR_PCONVERSION);
    leftArmMotor.enableVoltageCompensation(12);
    leftArmMotor.burnFlash();
    //leftArmEncoder.setPosition(0.0);
  }

  private void configTopShooterMotor() {
    topShooterMotor.restoreFactoryDefaults();
    //CANSparkMaxUtil.setCANSparkMaxBusUsage(topShooterMotor, Usage.kVelocityOnly);
    topShooterMotor.setSmartCurrentLimit(30);
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    topShooterMotor.enableVoltageCompensation(12);
    topShooterMotor.burnFlash();
  }

  private void configBottorShooterMotor() {
    bottomShooterMotor.restoreFactoryDefaults();
    //CANSparkMaxUtil.setCANSparkMaxBusUsage(bottomShooterMotor, Usage.kVelocityOnly);
    bottomShooterMotor.setSmartCurrentLimit(30);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.enableVoltageCompensation(12);
    bottomShooterMotor.burnFlash();
  }

  private void configIntakeMotor() {
    intakeMotor.restoreFactoryDefaults();
    //CANSparkMaxUtil.setCANSparkMaxBusUsage(intakeMotor, Usage.kVelocityOnly);
    intakeMotor.setSmartCurrentLimit(30);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.enableVoltageCompensation(12);
    intakeMotor.burnFlash();
  }

  public void moveArm(double pivotSpeed){
    rightArmMotor.set(pivotSpeed * -2.0);
    leftArmMotor.set(pivotSpeed * 2.0);
  }

  public void intake(double intakeSpeed){
    intakeMotor.set(intakeSpeed * 2);
  }

  public void shooter(double shooterSpeed){
    topShooterMotor.set(shooterSpeed);
    bottomShooterMotor.set(shooterSpeed);
  }
}
