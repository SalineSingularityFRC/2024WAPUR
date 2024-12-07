package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeMotor1;
  private CANSparkMax intakeMotor2;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public IntakeSubsystem() {

    intakeMotor1 = new CANSparkMax(Constants.CanId.Arm.Motor.INTAKE1, MotorType.kBrushless);
    intakeMotor1.restoreFactoryDefaults();
    m_pidController = intakeMotor1.getPIDController();
    m_encoder = intakeMotor1.getEncoder();

    intakeMotor2 = new CANSparkMax(Constants.CanId.Arm.Motor.INTAKE2, MotorType.kBrushless);
    intakeMotor2.restoreFactoryDefaults();
    intakeMotor2.follow(intakeMotor1);
    intakeMotor2.setInverted(true);

    // PID coefficients
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    setBrakeMode();
  }

  /**
   * @param speed Desired speed between [-1,1]
   */
  public void setIntakeSpeed(double speed) {
    m_pidController.setReference(speed, ControlType.kDutyCycle);
  }

  /**
   * @return Speed of NEO between [-1,1]
   */
  public double getIntakeSpeed() {
    //5676 is the free speed of the NEO in RPM
    return m_encoder.getVelocity() / 5676.0;
  }

  public void setBrakeMode() {
    intakeMotor1.setIdleMode(IdleMode.kBrake);
    intakeMotor2.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    intakeMotor1.setIdleMode(IdleMode.kCoast);
    intakeMotor2.setIdleMode(IdleMode.kCoast);
  }

  public Command setBrake() {
    return runOnce(
        () -> {
          setBrakeMode();
          
        });
  }
  public Command setCoast() {
    return runOnce(
        () -> {
          setCoastMode();
          
        });
  }
  public Command stopIntaking() {
    return runOnce(
        () -> {
          intakeMotor1.stopMotor();
          
        });
  }

  public Command autonStopIntaking(){
    return new FunctionalCommand(
    () -> {

    }, 
    () -> {
      intakeMotor1.stopMotor();
    },
    (_unused) -> {
      
    },
    () -> {
      return getIntakeSpeed() <= 5;
    },
    this
    );
  }
  

  public Command reverseIntake() {
    return run(
        () -> {
          setIntakeSpeed(-Constants.Speed.INTAKE);
        });
  }
  public Command startIntake() {
    return run(
        () -> {
          setIntakeSpeed(Constants.Speed.INTAKE);
        });
  }

public Command intakeContinue() {
    return run(
        () -> {
       
      });
  }
}
