package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
public class AutoCommands {
    public static Command driveDistanceCommand(double distance, Direction direction){
    return new FunctionalCommand(
      () -> RobotContainer.drivetrain.resetPosition(),
      () -> RobotContainer.drivetrain.driveDistance(distance, direction),
      (interrupt) -> RobotContainer.drivetrain.tankDrive(0, 0),
      () -> Math.abs(RobotContainer.drivetrain.getPosition()) >= Math.abs(RobotContainer.drivetrain.targetPosition) - 0.1,
      RobotContainer.drivetrain
    );
    }
}
