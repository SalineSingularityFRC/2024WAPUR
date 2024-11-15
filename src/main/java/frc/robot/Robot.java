// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SwerveClasses.SwerveOdometry;
// import au.grapplerobotics.LaserCan;
// import au.grapplerobotics.LaserCan.Measurement;
// import au.grapplerobotics.ConfigurationFailedException;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
    
  private Limelight lime;
  // private LaserCan laserCan1;
  // private LaserCan laserCan2;

  @Override
  public void robotInit() {

    // Required to allow power to the switchable port on the power distrubution hub and allow sensor
    // to use max power    

    m_robotContainer =
        new RobotContainer();
    lime = m_robotContainer.lime;
    // laserCan1 = m_robotContainer.laserCan1;
    // laserCan2 = m_robotContainer.laserCan2;

    // try {
    //   laserCan1.setRangingMode(LaserCan.RangingMode.SHORT);
    //   laserCan1.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
    //   laserCan1.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_100MS);
    //   laserCan2.setRangingMode(LaserCan.RangingMode.SHORT);
    //   laserCan2.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
    //   laserCan2.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_100MS);
    // } catch (ConfigurationFailedException e) {
    //   System.out.println("Configuration failed! " + e);
    // }
   
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
     m_robotContainer.drive.odometry.position();

    lime.update();
    lime.getDistanceToTagInFeet();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
   
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

  

    if (m_autonomousCommand != null) {

      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.drive.visionUpdateCommand();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // m_robotContainer.shooter.stopShooting();
    //robotSubsystem.setCoastMode();
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
