package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    public TalonFX l_motor, r_motor, on_motor, off_motor;
    
    public Climber() {
        l_motor = new TalonFX(12);
        r_motor = new TalonFX(13);
        on_motor = new TalonFX(3);
        off_motor= new TalonFX(2);
        
        
    }

    public void setVolts(double volts) {
        SmartDashboard.putNumber("climber speed", volts);
        l_motor.setControl(new VoltageOut(volts));
        r_motor.setControl(new VoltageOut(-volts));
    }

    @Override
    public void periodic() {
        
    }
}
