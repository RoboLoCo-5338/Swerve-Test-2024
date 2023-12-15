package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Effector extends SubsystemBase{

    private CANSparkMax effectorMotor;
    //public RelativeEncoder effectorEncoder;

    
    public Effector() {
        effectorMotor = new CANSparkMax(Constants.DriveConstants.kEffectorCanId, MotorType.kBrushless);
        //effectorMotor.setIdleMode(IdleMode.kBrake);
       // effectorMotor.setSmartCurrentLimit(25);
       // effectorEncoder = effectorMotor.getEncoder();
    }

    public void effectorForward() {
        SmartDashboard.putString("effector works", "yes");
        effectorMotor.set(0.8);
        
    }

    public void effectorReverse(){
        effectorMotor.set(-0.8);
    }

    public void effectorStop() {
        effectorMotor.set(0);
    }

}
