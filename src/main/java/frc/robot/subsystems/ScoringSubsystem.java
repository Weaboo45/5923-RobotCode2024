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

    private SparkPIDController rightArmController, leftArmController;

    public ScoringSubsystem() {
    
        //arm motors
        leftArmMotor = new CANSparkMax(Constants.leftArmMotorID, MotorType.kBrushless);
        rightArmMotor = new CANSparkMax(Constants.rightArmMotorID, MotorType.kBrushless);

        //arm encoders
        leftArmEncoder = leftArmMotor.getEncoder();
        rightArmEncoder = rightArmMotor.getEncoder();
        
        //shooter motors
        topShooterMotor = new CANSparkMax(Constants.topShooterMotorID, MotorType.kBrushless);
        bottomShooterMotor = new CANSparkMax(Constants.bottomShooterMotorID, MotorType.kBrushless);
        
        //intake motor
        intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushed);

        //PID controller
        rightArmController = rightArmMotor.getPIDController();
        leftArmController = leftArmMotor.getPIDController();

        //motor configs
        configLeftArmMotor();
        configRightArmMotor();
        configBottorShooterMotor();
        configTopShooterMotor();
        configIntakeMotor();
  }

  private void configRightArmMotor() {
    rightArmMotor.restoreFactoryDefaults();
    rightArmMotor.setSmartCurrentLimit(15);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmEncoder.setPositionConversionFactor(Constants.ARM_MOTOR_PCONVERSION);
    rightArmEncoder.setVelocityConversionFactor(Constants.ARM_MOTOR_VCONVERSION);
    
    rightArmController.setPositionPIDWrappingEnabled(true);
    rightArmController.setPositionPIDWrappingMinInput(-180.0);
    rightArmController.setPositionPIDWrappingMaxInput(180.0);
    rightArmController.setP(Constants.ROTATE_KP);
    rightArmController.setI(Constants.ROTATE_KI);
    rightArmController.setD(Constants.ROTATE_KD);
    rightArmController.setFF(0.0);
    
    rightArmMotor.enableVoltageCompensation(12);
    rightArmMotor.burnFlash();
    rightArmEncoder.setPosition(0);
  }

  private void configLeftArmMotor() {
    leftArmMotor.restoreFactoryDefaults();
    leftArmMotor.setSmartCurrentLimit(15);
    leftArmMotor.setIdleMode(IdleMode.kBrake);

    leftArmEncoder.setVelocityConversionFactor(Constants.ARM_MOTOR_VCONVERSION);
    leftArmEncoder.setPositionConversionFactor(Constants.ARM_MOTOR_PCONVERSION);

    leftArmController.setPositionPIDWrappingEnabled(true);
    leftArmController.setPositionPIDWrappingMinInput(-180.0);
    leftArmController.setPositionPIDWrappingMaxInput(180.0);
    leftArmController.setP(Constants.ROTATE_KP);
    leftArmController.setI(Constants.ROTATE_KI);
    leftArmController.setD(Constants.ROTATE_KD);
    leftArmController.setFF(0.0);
    
    leftArmMotor.enableVoltageCompensation(12);
    leftArmMotor.burnFlash();
    leftArmEncoder.setPosition(0.0);
  }

  private void configTopShooterMotor() {
    topShooterMotor.restoreFactoryDefaults();
    topShooterMotor.setSmartCurrentLimit(30);
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    topShooterMotor.enableVoltageCompensation(12);
    topShooterMotor.burnFlash();
  }

  private void configBottorShooterMotor() {
    bottomShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.setSmartCurrentLimit(30);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.enableVoltageCompensation(12);
    bottomShooterMotor.burnFlash();
  }

  private void configIntakeMotor() {
    intakeMotor.restoreFactoryDefaults();
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
