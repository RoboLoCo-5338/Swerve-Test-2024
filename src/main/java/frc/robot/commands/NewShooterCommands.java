package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;

public class NewShooterCommands {
  
    public static Command shooterReverse() {
      return new InstantCommand(
        () -> RobotContainer.m_shooterSubsystem.shooterReverse(),
        RobotContainer.m_shooterSubsystem
      );
    }
    
    public static Command shooterForward() {
      return new InstantCommand(
        () -> RobotContainer.m_shooterSubsystem.shooterForward(),
        RobotContainer.m_shooterSubsystem
      );

    }

    public static Command shooterStop() {
      return new InstantCommand(
        () -> RobotContainer.m_shooterSubsystem.shooterStop(),
        RobotContainer.m_shooterSubsystem
      );
    }

   
}
    