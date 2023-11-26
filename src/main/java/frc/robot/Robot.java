// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  // create joystick
  Joystick xBoxController = new Joystick(0);

  // tank drive
  private final TalonSRX Right_Drive_Motor_1 = new TalonSRX(Constants.Right_Drive_1);
  private final TalonSRX Right_Drive_Motor_2 = new TalonSRX(Constants.Right_Drive_2);
  private final TalonSRX Left_Drive_Motor_1 = new TalonSRX(Constants.Left_Drive_1);
  private final TalonSRX Left_Drive_Motor_2 = new TalonSRX(Constants.Left_Drive_2);

  // elevator
  private final TalonSRX Elevator_Motor_1 = new TalonSRX(Constants.Elevator_1);
  private final TalonSRX Elevator_Motor_2 = new TalonSRX(Constants.Elevator_2);

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    // tank drive
    Right_Drive_Motor_2.follow(Right_Drive_Motor_1);
    Left_Drive_Motor_2.follow(Left_Drive_Motor_1);

    Right_Drive_Motor_1.setInverted(false);
    Left_Drive_Motor_1.setInverted(false);

    Right_Drive_Motor_2.setInverted(InvertType.OpposeMaster);
    // Left drive only works with followmaster
    Left_Drive_Motor_2.setInverted(InvertType.FollowMaster);

    // elevator
    Elevator_Motor_2.follow(Elevator_Motor_1);

    Elevator_Motor_1.setInverted(false);
    Elevator_Motor_2.setInverted(InvertType.OpposeMaster);

    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Tank Drive --------------------------------> move to subsystem later
    double xBoxLeftYAxis = xBoxController.getRawAxis(1);
    double xBoxRightYAxis = xBoxController.getRawAxis(5);

    double LeftYAxis = xBoxLeftYAxis;
    double RightYAxis = xBoxRightYAxis;

    if (Constants.Deadzone_Factor >= Math.abs(xBoxLeftYAxis - 0)) {
      LeftYAxis = 0.0;
    }
    if (Constants.Deadzone_Factor >= Math.abs(xBoxRightYAxis - 0)) {
      RightYAxis = 0.0;
    }

    Left_Drive_Motor_1.set(ControlMode.PercentOutput, -LeftYAxis);
    Right_Drive_Motor_1.set(ControlMode.PercentOutput, -RightYAxis);

    // elevator
    double xBoxLeftTrigger = xBoxController.getRawAxis(2);
    double xBoxRightTrigger = xBoxController.getRawAxis(3);

    double xBoxTrigger = 0.0;

    if (Constants.Deadzone_Factor <= Math.abs(xBoxRightTrigger - 0)) {
      xBoxTrigger = xBoxRightTrigger;
    }
    if (Constants.Deadzone_Factor <= Math.abs(xBoxLeftTrigger - 0)) {
      xBoxTrigger = -xBoxLeftTrigger;

    }
    
    Elevator_Motor_1.set(ControlMode.PercentOutput, xBoxTrigger);


  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}