// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SmallArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.DriveController;
import frc.robot.commands.RumbleCommandStart;
import frc.robot.commands.RumbleCommandStop;
import frc.robot.commands.toSpeaker;
import frc.robot.commands.Teleop.SensorIntake;
import au.grapplerobotics.LaserCan;

public class RobotContainer {

    private SwerveSubsystem drive;
    private ArmSubsystem bigArm;
    private SmallArmSubsystem smallArm;
    protected LaserCan laserCan1;
    protected LaserCan laserCan2;
    //private Limelight lime;
    private IntakeSubsystem intake;
    private CommandXboxController driveController;
    private CommandXboxController armController;
    private SendableChooser<String> pathAutonChooser;

    protected RobotContainer() {

        //lime = new Limelight();
        bigArm = new ArmSubsystem();
        smallArm = new SmallArmSubsystem();
        drive = new SwerveSubsystem();
        intake = new IntakeSubsystem();

        driveController = new CommandXboxController(Constants.Gamepad.Controller.DRIVE);
        armController = new CommandXboxController(Constants.Gamepad.Controller.ARM);

        laserCan1 = new LaserCan(Constants.CanId.LaserCan.SENSOR1);
        laserCan2 = new LaserCan(Constants.CanId.LaserCan.SENSOR2);

        configureBindings();

        NamedCommands.registerCommand("StopDriving", drive.stopDriving());
        NamedCommands.registerCommand("RumbleCommantStart", new RumbleCommandStart(driveController));
        NamedCommands.registerCommand("RumbleCommantStop", new RumbleCommandStop(driveController));

        this.pathAutonChooser = new SendableChooser<String>();

        this.pathAutonChooser.setDefaultOption("WAPUR Auto Right", "WAPUR Auto Right");
        this.pathAutonChooser.addOption("WAPUR Auto Left", "WAPUR Auto Left");
        SmartDashboard.putData("Auton Choices", pathAutonChooser);
    }

    private void configureBindings() {
        intake.setDefaultCommand(intake.stopIntaking());
        bigArm.setDefaultCommand(bigArm.maintainArm());
        smallArm.setDefaultCommand(smallArm.maintainSmallArm());

        driveController.x().whileTrue(drive.resetGyroCommand());

        //Arm Controller
        //Intake for arm controller
        armController.a()
                .whileTrue(new SensorIntake(intake, laserCan1, laserCan2)
                .andThen(intake.stopIntaking().alongWith(new RumbleCommandStart(armController))));
        armController.a().onFalse(new RumbleCommandStop(armController));
        armController.b().whileTrue(intake.reverseIntake());
        //armController.a().onFalse(new ReverseIntakeCommand(intake));

        //driveController.b().whileTrue(
                //new toSpeaker(drive, lime)
        //);

        // Manual bigArm movement
        armController.povUp()
                .and(bigArm::isNotAtTop)
                .whileTrue(bigArm.moveArmForward());
        armController.leftBumper()
                .and(bigArm::isNotAtTop)
                .whileTrue(bigArm.moveArmForwardFast());
        armController.rightBumper()
                .and(bigArm::isNotAtTop)
                .whileTrue(bigArm.moveArmForwardFaster());

        armController.povDown()
                .and(bigArm::isNotAtBottom)
                .whileTrue(bigArm.moveArmBackwards());
        armController.povLeft()
                .whileTrue(smallArm.moveSmallArmForward());
        armController.povRight()
                .whileTrue(smallArm.moveSmallArmBackwards());
        
        //driveController.povRight().onTrue(drive.xMode());

        drive.setDefaultCommand(
                new DriveController(drive, () -> {
                        if (driveController.getRightX() < 0) {
                            return -1.0 * driveController.getRightX() * driveController.getRightX();
                        }

                        return driveController.getRightX() * driveController.getRightX();
                }, () -> {
                        if (driveController.getLeftY() < 0) {
                            return -1.0 * driveController.getLeftY() * driveController.getLeftY();
                        }

                        return driveController.getLeftY() * driveController.getLeftY();
                }, () -> {
                        if (driveController.getLeftX() < 0) {
                                return -1.0 * driveController.getLeftX() * driveController.getLeftX();
                        }

                        return driveController.getLeftX() * driveController.getLeftX();
                },
                        0.9, 1.5));
    }

    protected Command getAutonomousCommand() {
        return new PathPlannerAuto(this.pathAutonChooser.getSelected());
    }

    protected void updateOdometry() {
        this.drive.updateOdometry();
    }

}
