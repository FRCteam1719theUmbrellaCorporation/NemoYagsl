// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveDrive;
import java.util.function.BooleanSupplier;
import javax.print.attribute.standard.MediaSize.NA;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.LimeLightExtra;
//import frc.robot.subsystems.*;
import frc.robot.subsystems.Elevator.ElevatorSubsytem;
import frc.robot.subsystems.Elevator.ElevatorSubsytem.HeightLevels;
import frc.robot.subsystems.Elevator.EndEffectorSubsytem;
//import frc.robot.subsystems.intake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;
import frc.robot.subsystems.intake.CoralIntakeSubsystem.IntakePosition;
//import frc.robot.commands.*;
// import frc.robot.commands.Elevator.ElevatorPIDMoveCommand;
//import frc.robot.commands.Intake.AlgaeIntakeWheelsCommand;
import frc.robot.commands.Intake.CoralIntakeWheelsCommand;
import frc.robot.commands.Intake.CoralPivotPIDCommand;
import frc.robot.commands.outake.EndEffectorPIDCommand;
import frc.robot.commands.outake.IntakeCoralEndeffector;
//import frc.robot.commands.Intake.AlgaePivotPIDCommand;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed



  
  //Orinal port are driverXBox = 1, driverXBox2 = 0

  final CommandXboxController driverXbox = new CommandXboxController(0);

  final CommandXboxController driverXbox2 = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/nemo"));
  
  private final ElevatorSubsytem m_ElevatorSubsytem = new ElevatorSubsytem();
  private final CoralIntakeSubsystem m_CoralIntakeSubsystem = new CoralIntakeSubsystem();
  //private final AlgaeIntakeSubsystem m_AlgaeIntakeSubsystem = new AlgaeIntakeSubsystem();
  private final EndEffectorSubsytem m_EndEffectorSubsytem = new EndEffectorSubsytem();



  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getYButtonPressed,
                                                                 driverXbox.getHID()::getAButtonPressed,
                                                                 driverXbox.getHID()::getXButtonPressed,
                                                                 driverXbox.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);  

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  // Command algaeAngleSetter = new AlgaePivotPIDCommand(m_AlgaeIntakeSubsystem);
  // Command algaeWheels = new AlgaeIntakeWheelsCommand(m_AlgaeIntakeSubsystem);
  Command coralAngleSetter = new CoralPivotPIDCommand(m_CoralIntakeSubsystem);
  EndEffectorPIDCommand endEffDefaultCmd = new EndEffectorPIDCommand(m_EndEffectorSubsytem, m_ElevatorSubsytem);
  // Command intakeCoral = new IntakeCoralEndeffector(endEffDefaultCmd);

  CoralIntakeWheelsCommand coralWheels = new CoralIntakeWheelsCommand(m_CoralIntakeSubsystem);

  Command CoralDrive = new ParallelCommandGroup(
    new InstantCommand(()-> m_CoralIntakeSubsystem.setPosition(IntakePosition.DRIVING)),
    coralWheels.stopMotors());

  Command CoralFloor = new SequentialCommandGroup(
    new InstantCommand(()-> m_CoralIntakeSubsystem.setPosition(IntakePosition.FLOOR)),
    new WaitUntilCommand(()->MathUtil.isNear(CoralArmConstants.coral_floorintake_pos, m_CoralIntakeSubsystem.doubleMeasurement(), 0.005)),
    coralWheels.turnMotor(CoralArmConstants.coral_intake_floor_speed));

  Command CoralHumanPlayer =  new SequentialCommandGroup(
    new InstantCommand(()->m_CoralIntakeSubsystem.setPosition(IntakePosition.HUMAN_STATION)),
    //new WaitUntilCommand(()->MathUtil.isNear(CoralArmConstants.coral_humanstatione_pos, m_CoralIntakeSubsystem.doubleMeasurement(), 0.005)),
    coralWheels.turnMotor(CoralArmConstants.coral_intake_humanStation_speed));
  
  Command L1 = new SequentialCommandGroup(
    new InstantCommand(()-> m_CoralIntakeSubsystem.setPosition(IntakePosition.REEF)),
    new WaitUntilCommand(()->MathUtil.isNear(CoralArmConstants.coral_reef_l1, m_CoralIntakeSubsystem.doubleMeasurement(), 0.005)),
    coralWheels.turnMotor(CoralArmConstants.coral_outtake_reef_speed));
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  int i = 0;
  
  

  public RobotContainer()
  {
    // Pigeon2 m_gyro = new Pigeon2(2);
    // new LimeLightExtra(drivebase, m_gyro);

    Rotation3d sd = drivebase.getSwerveDrive().imuReadingCache.getValue();
    // LimelightHelpers.SetRobotOrientation(null, m_gyro.getYaw().getValueAsDouble(), m_gyro.getRate(), m_gyro.getPitch().getValueAsDouble(), 0, m_gyro.getRoll().getValueAsDouble(), 0);
    LimelightHelpers.SetIMUMode(null, 1);
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("center", drivebase.centerModulesCommand().withTimeout(0.5));
    NamedCommands.registerCommand("CoralDrive", CoralDrive);
    NamedCommands.registerCommand("CoralFloor", CoralFloor);
    NamedCommands.registerCommand("CoralHumanPlayer", CoralHumanPlayer);
    NamedCommands.registerCommand("CoralL1", L1);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings()
  {

   
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
                                driveFieldOrientedAnglularVelocity:
                                driveFieldOrientedAnglularVelocitySim);
   
    //m_AlgaeIntakeSubsystem.setDefaultCommand(algaeAngleSetter);
    m_CoralIntakeSubsystem.setDefaultCommand(coralAngleSetter);
    m_EndEffectorSubsytem.setDefaultCommand(endEffDefaultCmd);
    
    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {

     /* //Algae intake wheels to spin
      driverXbox2.a().onTrue(
        new AlgaeIntakeWheelsCommand(m_AlgaeIntakeSubsystem).turnMotor(1).withTimeout(1) //Test
      );
      
      driverXbox2.a().onFalse(
        new AlgaeIntakeWheelsCommand(m_AlgaeIntakeSubsystem).stopMotors() //Test
      );
*/
      //Coral intake wheels to spin
      driverXbox2.a().whileTrue(
        CoralFloor
      );
      driverXbox2.a().onFalse(
        CoralDrive
      );

      //Algae move to setpoint
      // driverXbox2.y().whileTrue(
      //   new SequentialCommandGroup(
      //     new InstantCommand(()->
      //     m_CoralIntakeSubsystem.setPosition(IntakePosition.HUMAN_STATION)
      //     ),
      //     new WaitUntilCommand(()->MathUtil.isNear(CoralArmConstants.coral_humanstatione_pos, m_CoralIntakeSubsystem.doubleMeasurement(), 0.005)),
      //     coralWheels.turnMotor(CoralArmConstants.coral_intake_humanStation_speed)
      //   )
      // );

      // driverXbox2.y().onFalse(
      //   new ParallelCommandGroup(
      //     new InstantCommand(()->
      //     m_CoralIntakeSubsystem.setPosition(IntakePosition.DRIVING)),
      //     coralWheels.stopMotors()
      //   )
      // );
      
      // //Coral move to reef l1
      // driverXbox2.x().onTrue(
      //   new SequentialCommandGroup(
      //     new InstantCommand(()->
      //     m_CoralIntakeSubsystem.setPosition(IntakePosition.REEF)
      //     ),
      //     new WaitUntilCommand(()->MathUtil.isNear(CoralArmConstants.coral_reef_l1, m_CoralIntakeSubsystem.doubleMeasurement(), 0.005)),
      //     coralWheels.turnMotor(CoralArmConstants.coral_outtake_reef_speed)
      //   )
      // );
      // driverXbox2.y().onTrue(
      //   new InstantCommand(() -> {
      //     m_ElevatorSubsytem.setSetpoint(20);

      //   })
      // );

      // driverXbox2.y().onFalse(
      //   new InstantCommand(()->
      //   m_ElevatorSubsytem.setSetpoint(50)
      //   )
      // );
      //Coral move to setpoint
       driverXbox2.x().whileTrue(
        new InstantCommand(() -> {
        m_CoralIntakeSubsystem.setSetpoint(.1);
        })
       );
      driverXbox2.x().whileTrue(
        CoralHumanPlayer
      );
      driverXbox2.x().onFalse(
        CoralDrive
      );
      
      //Coral move to reef l1
      driverXbox2.b().onTrue(
        L1
      );
      driverXbox2.b().onFalse(
        CoralDrive
      );

      // driverXbox2.b().whileTrue(
      //   new InstantCommand(() -> {
      //     m_AlgaeIntakeSubsystem.setSetpoint(0.2);
      //   })
      // );
      // driverXbox2.b().onFalse(
      //   new InstantCommand(() -> {
      //     m_AlgaeIntakeSubsystem.setSetpoint(0.1);
      //   })
      // );


      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                         );
      // driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      
      driverXbox2.start().onTrue(
        IntakeCoralEndeffector.intake(endEffDefaultCmd)
        // new SequentialCommandGroup(
          // new InstantCommand(()-> {
          //   AutoBuilder.buildAuto("uto").schedule();
          // }))
          );
    }}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    LimelightHelpers.SetIMUMode(null, 2);

    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("uto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
