package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hook extends SubsystemBase {
    private CANSparkMax hookMotor;
    public RelativeEncoder hookEncoder;

    public static double hookChange = 0;
    public Hook(){
        hookMotor = new CANSparkMax(Constants.DriveConstants.kBunnyArmCanId, MotorType.kBrushless);
        hookMotor.setIdleMode(IdleMode.kBrake);
        hookEncoder = hookMotor.getEncoder();
        hookMotor.setSmartCurrentLimit(40);
    }
        
    public void resetHook() {
        hookEncoder.setPosition(0);

    }

    public double getHookPosition() {
        return hookEncoder.getPosition();
    
    }

    public void stopHook() {
        hookMotor.set(0);
    }

    public void moveHook(double speed) {
        hookMotor.set(speed);
    }

 }