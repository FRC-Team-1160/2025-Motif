// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DriveTrain;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrainRealIO extends DriveTrain {

  private AHRS m_gyro;

  public DriveTrainRealIO(){

    // m_gyro = new AHRS(AHRS.NavXComType.kI2C);
    // m_gyro.zeroYaw();
  
  }

  public SwerveModule initializeModule(int drive_port, int steer_port, int sensor_port){
    return new SwerveModuleRealIO(drive_port, steer_port, sensor_port);
  }

  public Rotation2d getGyroAngle(){
    if (m_gyro != null){
      return Rotation2d.fromDegrees(-m_gyro.getAngle()); //gyro reports CW positive, negate to return CCW positive
    } else if (m_odom_pose != null) {
      SmartDashboard.putNumber("vibe check", Math.random());
      return m_odom_pose.getRotation();
    } else {
      SmartDashboard.putNumber("vibe check 2", Math.random());
      return new Rotation2d();
    }
  }

  @Override
  public void periodic() {
    //the child periodic() method overrides the parent class periodic(), which has to be explicitly called
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
