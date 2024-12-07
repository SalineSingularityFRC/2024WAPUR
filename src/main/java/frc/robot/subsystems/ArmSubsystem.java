package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Limelight;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class ArmSubsystem extends SubsystemBase {

  // Talon Setup
  private TalonFX baseArmMotor1;
  private TalonFX baseArmMotor2;

  private PositionVoltage positionTargetPreset = new PositionVoltage(0).withSlot(0).withEnableFOC(true);

  private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(1).withEnableFOC(true);
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0.2);
  private TalonFXConfiguration talonFXConfigsPreset = new TalonFXConfiguration();
  private MotionMagicConfigs motionMagicConfigsPresets;
  private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
  

  private final double manualBigP = 1;
  private final double manualBigI = 0;
  private final double manualBigD = 0;
  private final double manualBigS = 0.03;

  // OLD PID CONSTANTS
  private final double slot0P = 0.4;
  private final double slot0I = 0;
  private final double slot0D = 0.1;
  private final double slot0S = 0.0;

  public double baseArmMotorPosition;
  public double smallArmMotorPosition;

  // Neo Setup
  private CANSparkMax smallArmMotor;
  private SparkPIDController smallArmPidController;
  private RelativeEncoder smallArmEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  public ArmSubsystem() {

    // Talon Portion
    baseArmMotor1 = new TalonFX(Constants.CanId.Arm.Motor.BASE1, Constants.Canbus.DEFAULT);
    baseArmMotor2 = new TalonFX(Constants.CanId.Arm.Motor.BASE2, Constants.Canbus.DEFAULT);

    // Factory reset before applying configs
    baseArmMotor1.getConfigurator().apply(new TalonFXConfiguration());
    baseArmMotor1.getConfigurator().apply(new TalonFXConfiguration());

    baseArmMotor2.setControl(new Follower(Constants.CanId.Arm.Motor.BASE1, true));

    HardwareLimitSwitchConfigs limitSwitchConfigs = new HardwareLimitSwitchConfigs();
    limitSwitchConfigs.ReverseLimitAutosetPositionEnable = true;
    limitSwitchConfigs.ReverseLimitAutosetPositionValue = 0;
    limitSwitchConfigs.ForwardLimitAutosetPositionEnable = false;

    Slot0Configs slot0ConfigsBig = new Slot0Configs();
    slot0ConfigsBig.kP = slot0P;
    slot0ConfigsBig.kI = slot0I;
    slot0ConfigsBig.kD = slot0D;
    slot0ConfigsBig.kS = slot0S;

    Slot1Configs slot1ConfigsBig = new Slot1Configs();
    slot1ConfigsBig.kP = manualBigP;
    slot1ConfigsBig.kI = manualBigI;
    slot1ConfigsBig.kD = manualBigD;
    slot1ConfigsBig.kS = manualBigS;

    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = 30;

    baseArmMotor1.getConfigurator().apply(slot0ConfigsBig);
    baseArmMotor1.getConfigurator().apply(slot1ConfigsBig);
    baseArmMotor1.getConfigurator().apply(limitSwitchConfigs);
    baseArmMotor1.getConfigurator().apply(currentLimitsConfigs);

    motionMagicConfigsPresets = talonFXConfigsPreset.MotionMagic;
    motionMagicConfigsPresets.MotionMagicCruiseVelocity = 40 / 30;
    motionMagicConfigsPresets.MotionMagicAcceleration = 100 / 80;
    motionMagicConfigsPresets.MotionMagicJerk = 900 / 60;

    baseArmMotor1.getConfigurator().apply(motionMagicConfigsPresets);

    // Global Position Value
    baseArmMotorPosition = baseArmMotor1.getPosition().getValue();

    // Neo Portion
    smallArmMotor = new CANSparkMax(Constants.CanId.Arm.Motor.SMALL, MotorType.kBrushless);
    smallArmMotor.restoreFactoryDefaults();
    smallArmPidController = smallArmMotor.getPIDController();
    smallArmEncoder = smallArmMotor.getEncoder();

    // PID coefficients
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    smallArmPidController.setP(kP);
    smallArmPidController.setI(kI);
    smallArmPidController.setD(kD);
    smallArmPidController.setIZone(kIz);
    smallArmPidController.setFF(kFF);
    smallArmPidController.setOutputRange(kMinOutput, kMaxOutput);

    // Global Position Value
    smallArmMotorPosition = smallArmEncoder.getPosition();

    setBrakeMode();
  }

  public void setArmSpeed(double baseArmspeed, double smallArmspeed) {
    baseArmMotor1.setControl(dutyCycleOut.withOutput(baseArmspeed).withEnableFOC(true));
    baseArmMotorPosition = baseArmMotor1.getPosition().getValue();

    smallArmPidController.setReference(smallArmspeed, ControlType.kDutyCycle);
    smallArmMotorPosition = smallArmEncoder.getPosition();
  }

  public void setPosition(double baseArmAngle, double smallArmAngle) {
    baseArmMotor1.setControl(
        positionTargetPreset.withPosition(baseArmAngle).withFeedForward(0.1).withSlot(0));
    baseArmMotorPosition = baseArmAngle;

    smallArmPidController.setReference(smallArmAngle, ControlType.kPosition);
    smallArmMotorPosition = smallArmAngle;
  }

  public void setBrakeMode() {
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    baseArmMotor1.getConfigurator().apply(motorOutputConfigs);
    baseArmMotor2.getConfigurator().apply(motorOutputConfigs);

    smallArmMotor.setIdleMode(IdleMode.kBrake);
  }

  public void periodic() {

  }

  public Command stopArm() {
    return run(
        () -> {
          maintainArmPosition();
        });
  }

  public Command moveArmForward() {
    return run(
        () -> {
          setArmSpeed(Constants.Speed.ARMDUTYCYCLEUP,0);
        });
  }

  public Command moveArmBackwards() {
    return run(
        () -> {
          setArmSpeed(-Constants.Speed.ARMDUTYCYCLEDOWN,0);
        });
  }

  // RunOne or Run
  public Command ampTarget() {
    return runOnce(() -> {
      setPosition(Constants.Position.MainArm.AMP,0);
    });
  }

  public Command shootTarget() {
    return new FunctionalCommand(
        () -> {

        },
        () -> {
          setPosition(Constants.Position.MainArm.Speaker.FEET3,0);
        },
        (_unused) -> {

        },
        () -> {
          return Math.abs(Constants.Position.MainArm.Speaker.FEET3 - baseArmMotor1.getPosition().getValueAsDouble()) < 0.5;
        },
        this);
  }

  public Command autonShootTarget(double pos) {
    return new FunctionalCommand(
        () -> {

        },
        () -> {
          setPosition(pos,0);
        },
        (_unused) -> {

        },
        () -> {

          return Math.abs(pos - baseArmMotor1.getPosition().getValueAsDouble()) < 1;
        },
        this);
  }

  public Command limelightShootTarget(Limelight lime) {
    return new FunctionalCommand(
        () -> {
        
        },
        () -> {
          double distance = lime.getDistanceToTagInFeet();
          double toDriveDistance = 0;

          if (distance > 6) {
            toDriveDistance = 6;
          }
          else {
            toDriveDistance = distance;
          }
        
          double shootingPos = lime.getArmPositionFromDistance(toDriveDistance);
          SmartDashboard.putNumber("shootingPosLimelight", shootingPos);

          baseArmMotor1.setControl(
          positionTargetPreset.withPosition(shootingPos).withFeedForward(0.1).withSlot(0));
          baseArmMotorPosition = shootingPos;
        },
        (_unused) -> {

        },
        () -> {
          double distance = lime.getDistanceToTagInFeet();
          double toDriveDistance = 0;

          if (distance > 6) {
            toDriveDistance = 6;
          }
          else {
            toDriveDistance = distance;
          }
        
          double shootingPos = lime.getArmPositionFromDistance(toDriveDistance);
          return Math.abs(shootingPos- baseArmMotor1.getPosition().getValueAsDouble()) < 1;
        },
        this);
  }

  public Command pickupTarget() {
    return runOnce(() -> {
      setPosition(Constants.Position.MainArm.PICKUP,0);
    });
  }

  public Command maintainArm() {
    return run(() -> {
      maintainArmPosition();
    });
  }

  public boolean isNotAtBottom() {
    return baseArmMotor1.getReverseLimit().getValue() != ReverseLimitValue.ClosedToGround;
  }

  public boolean isAtBottom() {
    return baseArmMotor1.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  public boolean isNotAtTop() {
    return baseArmMotor1.getForwardLimit().getValue() != ForwardLimitValue.ClosedToGround;
  }

  public void maintainArmPosition() {
    // System.out.println("MAINTAIN");
    // baseArmMotor1.getConfigurator().apply(motionMagicConfigsPresets);
    baseArmMotor1.setControl(
        positionTargetPreset.withPosition(baseArmMotorPosition).withFeedForward(0.03 * 12).withSlot(0));
  }

  public double getPosition() {
    return (baseArmMotor1.getPosition().getValue() * 2 * Math.PI) / Constants.MotorGearRatio.ARM;
  }

  public Command goHome() {
    return new FunctionalCommand(
        () -> {

        },
        () -> {
          setArmSpeed(-Constants.Speed.HOME,0);
        },
        (_unused) -> {

        },
        this::isAtBottom,
        this);
  }
}
