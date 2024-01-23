package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewShooter extends SubsystemBase{
    private CANSparkMax shootMotor1;
    private CANSparkMax shootMotor2;

    public NewShooter() {
        shootMotor1 = new CANSparkMax(Constants.shootMotor1,MotorType.kBrushless);
        shootMotor2 = new CANSparkMax(Constants.shootMotor2,MotorType.kBrushless);
    }

    public void shooterForward() {
        shootMotor1.set(0.9);
        shootMotor2.set(-0.9);
    }

    public void shooterReverse() {
        shootMotor1.set(-0.3);
        shootMotor2.set(0.3);
    }

    public void shooterStop() {
        shootMotor1.set(0);
        shootMotor2.set(0);
    }
    
}
