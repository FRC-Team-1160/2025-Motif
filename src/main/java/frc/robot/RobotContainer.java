// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.Subsystems.DriveTrain.DriveTrainSimIO;


public class RobotContainer {
  private Joystick main_stick = new Joystick(Constants.IO.MAIN_PORT);
  private Joystick second_stick = new Joystick(Constants.IO.COPILOT_PORT);
  // private Joystick left_board = new Joystick(Constants.IO.LEFT_BOARD_PORT);
  private Joystick right_board = new Joystick(Constants.IO.RIGHT_BOARD_PORT);

  private Joystick simp_stick = new Joystick(2);

  public final DriveTrain m_drive = Robot.isReal() ? new DriveTrainRealIO() : new DriveTrainSimIO();

  public final Climber m_climber = new Climber();

  private final SendableChooser<Command> auto_chooser;

  public RobotContainer() {
    auto_chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", auto_chooser);
    configureBindings();
  }

  public void updateSwerve() {
    // m_drive.setSwerveDrive(
    //   (Math.abs(main_stick.getRawAxis(1)) < 0.1) ? 0 : 2.5 * main_stick.getRawAxis(1), 
    //   (Math.abs(main_stick.getRawAxis(0)) < 0.1) ? 0 : 2.5 * main_stick.getRawAxis(0), 
    //   (Math.abs(second_stick.getRawAxis(0)) < 0.1) ? 0 : Math.signum(second_stick.getRawAxis(0)) * 2.5
    //    * Math.pow(second_stick.getRawAxis(0), 2)
    //   );

    m_drive.setSwerveDrive(
      (Math.abs(simp_stick.getRawAxis(5)) < 0.1) ? 0 : 1.5 * -simp_stick.getRawAxis(5), 
      (Math.abs(simp_stick.getRawAxis(4)) < 0.1) ? 0 : 1.5 * -simp_stick.getRawAxis(4), 
      (Math.abs(simp_stick.getRawAxis(0)) < 0.2) ? 0 : Math.signum(simp_stick.getRawAxis(0)) * 1.5
        * Math.pow(simp_stick.getRawAxis(0), 2)
      );
  
  }

  private void configureBindings() {
    new JoystickButton(main_stick, 8).onTrue(
      new InstantCommand(m_drive::resetGyroAngle)
    );

    new JoystickButton(main_stick, 9).onTrue(
      new InstantCommand(m_drive::resetGyroAngle)
    );

    // new JoystickButton(main_stick, 7).toggleOnFalse(
    //   m_drive.musicCommand("test")
    // );

    // new JoystickButton(main_stick, 6).toggleOnFalse(
    //   m_drive.musicCommand("mario",2)
    // );

    // new JoystickButton(main_stick, 10).toggleOnFalse(
    //   m_drive.musicCommand("USA", 2)
    // );

    // new JoystickButton(main_stick, 11).toggleOnFalse(
    //   m_drive.musicCommand("datingStart", 4)
    // );

    // new JoystickButton(simp_stick, 1).toggleOnFalse(
    //   m_drive.musicCommand("test", 1)
    // );

    // new JoystickButton(simp_stick, 2).toggleOnFalse(
    //   m_drive.musicCommand("mario",2)
    // );

    // new JoystickButton(simp_stick, 3).toggleOnFalse(
    //   m_drive.musicCommand("USA", 2)
    // );

    // new JoystickButton(simp_stick, 4).toggleOnFalse(
    //   m_drive.musicCommand("datingStart", 4)
    // );

    new JoystickButton(simp_stick, 5).whileTrue(
      new StartEndCommand(
        () -> m_climber.setVolts(2), 
        () -> m_climber.setVolts(0), 
        m_climber));

    new JoystickButton(simp_stick, 6).whileTrue(
      new StartEndCommand(
        () -> m_climber.setVolts(-2), 
        () -> m_climber.setVolts(0), 
        m_climber));
    
    new JoystickButton(simp_stick, 8).onTrue(
      new InstantCommand(m_drive::resetGyroAngle)
    );
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
