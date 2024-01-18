// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d; //1/18/24
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.EffectorCommands;
import frc.robot.commands.HookCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Effector;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hook;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.choreo.lib.Choreo; //1/18/24
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static final Shooter m_shooter = new Shooter();
  public static final Hook m_hook = new Hook();
  public static final Intake intake = new Intake();
  public static final Effector m_effector = new Effector();


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
 // XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

private static Joystick controller2 = new Joystick(OIConstants.kOperatorControllerPort);

ChoreoTrajectory traj; //1/18/24
Field2d m_field = new Field2d();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    traj = Choreo.getTrajectory("Trajectory"); //1/18/24

    m_field.getObject("traj").setPoses(
      traj.getInitialPose(), traj.getFinalPose()
    );
    m_field.getObject("trajPoses").setPoses(
      traj.getPoses()
    );
    // SmartDashboard.putData(m_field);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    Trigger shooterForward = new Trigger(() -> controller2.getRawAxis(3) > 0.4);
    shooterForward.whileTrue(ShooterCommands.shooterForward());
    shooterForward.onFalse(ShooterCommands.shooterStop());

    
    Trigger shooterReverse = new Trigger(() -> controller2.getRawAxis(2) > 0.4);
    shooterReverse.whileTrue(ShooterCommands.shooterReverse());
    shooterReverse.onFalse(ShooterCommands.shooterStop());
 
    
    JoystickButton effectorForward = new JoystickButton(controller2, Constants.RBBUTTON);
    effectorForward.whileTrue(EffectorCommands.effectorForward());
    effectorForward.onFalse(EffectorCommands.effectorStop());

    
    JoystickButton effectorReverse = new JoystickButton(controller2, Constants.LBBUTTON);
    effectorReverse.whileTrue(EffectorCommands.effectorReverse());
    effectorReverse.onFalse(EffectorCommands.effectorStop());
    

      
JoystickButton hookUp = new JoystickButton(controller2, Constants.BBUTTON);
hookUp.whileTrue(HookCommands.moveUp());
hookUp.onFalse(HookCommands.stopHook());


JoystickButton hookDown = new JoystickButton(controller2, Constants.XBUTTON);
hookDown.whileTrue(HookCommands.moveDown());
hookDown.onFalse(HookCommands.stopHook());

JoystickButton intakeUp = new JoystickButton(controller2, Constants.ABUTTON);
intakeUp.whileTrue(IntakeCommands.intake(0.1));
intakeUp.onFalse(IntakeCommands.stopIntake());


JoystickButton intakeDown = new JoystickButton(controller2, Constants.YBUTTON);
intakeDown.whileTrue(IntakeCommands.intake(-0.1));
intakeDown.onFalse(IntakeCommands.stopIntake());



   // effectorForward.whileFalse(EffectorCommands.effectorStop());
    // if (m_operatorController.getRightBumperPressed()){
    //     EffectorCommands.effectorForward();
    // }
    // if (m_operatorController.getLeftBumperPressed()){
    //     EffectorCommands.effectorReverse();
    // }
    // if (m_operatorController.getBButtonPressed()){
    //     HookCommands.moveUp();
    // }
    // if (m_operatorController.getXButtonPressed()){
    //     HookCommands.moveDown();
    // }
    // if (m_operatorController.getRightTriggerAxis() > 0.1){
    //     ShooterCommands.shooterForward();
    // }
    // if (m_operatorController.getLeftTriggerAxis() > 0.1){
    //     ShooterCommands.shooterReverse();
    // }
    //IntakeCommands.intake(m_operatorController.getRightY());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
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

    var thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_robotDrive.resetOdometry(traj.getInitialPose());

    Command swerveCommand = Choreo.choreoSwerveCommand(
        traj, // Choreo trajectory from above
        m_robotDrive::getPose, // A function that returns the current field-relative pose of the robot: your
                               // wheel or vision odometry
        new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // PIDController for field-relative X
                                                                                   // translation (input: X error in meters,
                                                                                   // output: m/s).
        new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), // PIDController for field-relative Y
                                                                                   // translation (input: Y error in meters,
                                                                                   // output: m/s).
        thetaController, // PID constants to correct for rotation
                         // error
        (ChassisSpeeds speeds) -> m_robotDrive.drive( // needs to be robot-relative
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            false, true),
        true, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
        m_robotDrive // The subsystem(s) to require, typically your drive subsystem only
    );

    return Commands.sequence(
      Commands.runOnce(() -> m_robotDrive.resetOdometry(traj.getInitialPose())),
      swerveCommand,
      m_robotDrive.run(() -> m_robotDrive.drive(0, 0, 0, false, true))
  );
  }

  public void periodic() { //FOR CHOREO 1/18/24
    m_field.setRobotPose(m_robotDrive.getPose());
  }
}
