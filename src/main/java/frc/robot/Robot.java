// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final GenericHID mainJoy = new GenericHID(1);
  private Command m_autonomousCommand;
  private final TalonFX motorOne = new TalonFX(1);
  private final TalonFX motorTwo = new TalonFX(2);
  private final TalonFX motorThree = new TalonFX(3);
  private final TalonFX motorFour = new TalonFX(4);
  private final TalonFX motorFive = new TalonFX(5);
  private final TalonFX motorSix = new TalonFX(6);
  private final TalonFX motorSeven = new TalonFX(7);
  private final TalonFX motorEight = new TalonFX(8);


  private final RobotContainer m_robotContainer;

  public void fourMotorsVolts(double one, double two, double three, double four){
    motorTwo.setVoltage(one);
    motorFour.setVoltage(two);
    motorSix.setVoltage(three);
    motorEight.setVoltage(four);
  }

  
  public void turnL(){
    motorOne.setVoltage(2.0);
    motorThree.setVoltage(2.0);
    motorFive.setVoltage(2.0);
    motorSeven.setVoltage(2.0);
  }

  public void turnR(){
    motorOne.setVoltage(-2.0);
    motorThree.setVoltage(-2.0);
    motorFive.setVoltage(-2.0);
    motorSeven.setVoltage(-2.0);
  }
  
  public void turnOffG(){
   motorOne.setVoltage(0);
    motorThree.setVoltage(0);
    motorFive.setVoltage(0);
    motorSeven.setVoltage(0);
  }

  public void turnOffD(){
    motorTwo.setVoltage(0.0);
    motorFour.setVoltage(0.0);
    motorSix.setVoltage(0.0);
    motorEight.setVoltage(0.0);
  }
  
  private void binds(){
    new JoystickButton(mainJoy, 1)
      .whileTrue(new StartEndCommand(
          () -> fourMotorsVolts(2.0, 2.0, 2.0, 2.0),
          () ->  turnOffD()));

    
    new JoystickButton(mainJoy, 2)
      .onTrue(new StartEndCommand(
          () -> turnL(),
          () ->  turnOffG()));

    new JoystickButton(mainJoy, 3)
      .onTrue(new StartEndCommand(
          () -> turnR(),
          () ->  turnOffG()));
    
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
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

    binds();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

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