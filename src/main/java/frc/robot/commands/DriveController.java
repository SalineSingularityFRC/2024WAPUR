package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveController extends Command {
    private final SwerveSubsystem m_swerve;
    private final DoubleSupplier m_rotation, m_x, m_y;
    private double drive_multiplier;
    private double turn_multiplier;

    public DriveController(SwerveSubsystem swerve, DoubleSupplier rotation, 
        DoubleSupplier x, DoubleSupplier y, double drive_multiplier, double turn_multiplier) {

        this.drive_multiplier = drive_multiplier;
        this.turn_multiplier = turn_multiplier;
        m_swerve = swerve;
        m_rotation = rotation;
        m_x = x;
        m_y = y;
        addRequirements(swerve);
    }

    private double fixDecimalTo2Places(double number){
        return Math.round(number * 100.0) / 100.0;
    }

    public void execute() {
        SmartDashboard.putNumber("Input X", fixDecimalTo2Places(-m_x.getAsDouble()));
        SmartDashboard.putNumber("Input Y", fixDecimalTo2Places(-m_y.getAsDouble()));
        SmartDashboard.putNumber("Input Rotation", fixDecimalTo2Places(-m_rotation.getAsDouble()));
        m_swerve.drive(
                -m_rotation.getAsDouble() * turn_multiplier,
                -m_x.getAsDouble() * drive_multiplier,
                -m_y.getAsDouble() * drive_multiplier,
                true);
    }
  

    public boolean isFinished() {
        return false;
    }
}
