package frc.robot.subsystems;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
public class Auto {
    public static Command getAutonomousCommand() {
        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        //     AutoConstants.kMaxSpeedMetersPerSecond,
        //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //     // Add kinematics to ensure max speed is actually obeyed
        //     .setKinematics(DriveConstants.kDriveKinematics);
    
        // // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //     // Start at the origin facing the +X direction
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     // Pass through these two interior waypoints, making an 's' curve path
        //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        //     // End 3 meters straight ahead of where we started, facing forward
        //     new Pose2d(3, 0, new Rotation2d(0)),
        //     config);
    
        // var thetaController = new ProfiledPIDController(
        //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //     exampleTrajectory,
        //     m_robotDrive::getPose, // Functional interface to feed supplier
        //     DriveConstants.kDriveKinematics,
    
        //     // Position controllers
        //     new PIDController(AutoConstants.kPXController, 0, 0),
        //     new PIDController(AutoConstants.kPYController, 0, 0),
        //     thetaController,
        //     m_robotDrive::setModuleStates,
        //     m_robotDrive);
    
        // // Reset odometry to the starting pose of the trajectory.
        // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    
        // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true, false));
    
        //1/18/24 ---- CHOREO TESTING
    
      //   var thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
      //   thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
      //   m_robotDrive.resetOdometry(traj.getInitialPose());
    
      //   Command swerveCommand = Choreo.choreoSwerveCommand(
      //       traj, // Choreo trajectory from above
      //       m_robotDrive::getPose, // A function that returns the current field-relative pose of the robot: your
      //                              // wheel or vision odometry
      //       new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // PIDController for field-relative X
      //                                                                                  // translation (input: X error in meters,
      //                                                                                  // output: m/s).
      //       new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), // PIDController for field-relative Y
      //                                                                                  // translation (input: Y error in meters,
      //                                                                                  // output: m/s).
      //       thetaController, // PID constants to correct for rotation
      //                        // error
      //       (ChassisSpeeds speeds) -> m_robotDrive.drive( // needs to be robot-relative
      //           speeds.vxMetersPerSecond,
      //           speeds.vyMetersPerSecond,
      //           speeds.omegaRadiansPerSecond,
      //           false, true),
      //       () -> {
      //           return false;
      //         }, // Whether or not to mirror the path based on alliance (CAN ADD LOGIC TO DO THIS AUTOMATICALLY)
      //       m_robotDrive // The subsystem(s) to require, typically your drive subsystem only
      //   );
    
      //   return Commands.sequence(
      //     Commands.runOnce(() -> m_robotDrive.resetOdometry(traj.getInitialPose())),
      //     swerveCommand,
      //     m_robotDrive.run(() -> m_robotDrive.drive(0, 0, 0, false, true))
      // );
      DriverStation.Alliance ally = DriverStation.getAlliance();
        if (ally == DriverStation.Alliance.Red) {
          switch (AutoConstants.autoNum){
            case 1:
              return AutoCommands.leftRed();
            case 2:
              return AutoCommands.midRed();
            case 3:
              return AutoCommands.rightRed();
          }
        }
        else if (ally == DriverStation.Alliance.Blue) {
          switch (AutoConstants.autoNum){
            case 1:
              return AutoCommands.leftBlue();
            case 2:
              return AutoCommands.midBlue();
            case 3:
              return AutoCommands.rightBlue();
          }
        }
        else {
            return null;
        }
        return null;
      }
}
