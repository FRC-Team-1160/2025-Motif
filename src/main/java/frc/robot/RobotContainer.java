// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.Subsystems.DriveTrain.DriveTrainSimIO;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class RobotContainer {
  // private Joystick main_stick = new Joystick(Constants.IO.MAIN_PORT);
  // private Joystick second_stick = new Joystick(Constants.IO.COPILOT_PORT);
  // // private Joystick left_board = new Joystick(Constants.IO.LEFT_BOARD_PORT);
  // private Joystick right_board = new Joystick(Constants.IO.RIGHT_BOARD_PORT);


  

  private Joystick simp_stick = new Joystick(2); 

  public final DriveTrain m_drive = Robot.isReal() ? new DriveTrainRealIO() : new DriveTrainSimIO();

  // public final Climber m_climber = new Climber();

  //private final SendableChooser<Command> auto_chooser;

  /*
  public RobotContainer() {
    auto_chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", auto_chooser);
    configureBindings();
  }
    */

  public void updateSwerve() {
    double rightStickUpDown = simp_stick.getRawAxis(5);
    SmartDashboard.putNumber("joystick_axis_5", rightStickUpDown);

    double x_metersPerSecond = (Math.abs(simp_stick.getRawAxis(5)) < 0.1) ? 0 : 1.5 * -simp_stick.getRawAxis(5);
    SmartDashboard.putNumber("x_mps", x_metersPerSecond);

    double rightStickLeftRight = simp_stick.getRawAxis(4);
    SmartDashboard.putNumber("joystick_axis_4", rightStickLeftRight);

    double y_metersPerSecond = (Math.abs(simp_stick.getRawAxis(4)) < 0.1) ? 0 : 1.5 * -simp_stick.getRawAxis(4);
    SmartDashboard.putNumber("y_mps", y_metersPerSecond);

    double leftStickLeftRight = simp_stick.getRawAxis(0);
    double angle_radiansPerSecond =  (Math.abs(simp_stick.getRawAxis(0)) < 0.2) ? 0 : Math.signum(simp_stick.getRawAxis(0)) * 1.5
    * Math.pow(simp_stick.getRawAxis(0), 2);
    SmartDashboard.putNumber("axis_0", leftStickLeftRight);
    SmartDashboard.putNumber("angle", angle_radiansPerSecond);

    m_drive.setSwerveDrive(
      x_metersPerSecond, 
      y_metersPerSecond, 
      angle_radiansPerSecond
      );
  
  }

  private void configureBindings() {
    new JoystickButton(simp_stick, 8).onTrue(
      new InstantCommand(m_drive::resetGyroAngle)
    );

    new JoystickButton(simp_stick, 8).onTrue(
      new InstantCommand(m_drive::resetGyroAngle)
    );

    new JoystickButton(simp_stick,1).onTrue(
      new InstantCommand(() -> m_drive.setAngle(0))
    );

    new JoystickButton(simp_stick,2).onTrue(
      new InstantCommand(() -> m_drive.setAngle(90))
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

    // new JoystickButton(simp_stick, 4).whileTrue(
    //   new StartEndCommand(
    //     () -> m_climber.setVolts(2), 
    //     () -> m_climber.setVolts(0), 
    //     m_climber));

    // new JoystickButton(simp_stick, 1).whileTrue(
    //   new StartEndCommand(
    //     () -> m_climber.setVolts(-2), 
    //     () -> m_climber.setVolts(0), 
    //     m_climber));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
