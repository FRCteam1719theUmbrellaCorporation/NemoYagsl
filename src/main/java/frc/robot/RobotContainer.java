// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import utils.Reef.Level;
import utils.Reef.Location;
import frc.robot.subsystems.LimeLightExtra;
import frc.robot.subsystems.Elevator.ElevatorSubsytem;
import frc.robot.subsystems.Elevator.EndEffectorSubsytem;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;
import frc.robot.subsystems.intake.CoralIntakeSubsystem.IntakePosition;
import frc.robot.commands.Intake.CoralIntakeWheelsCommand;
import frc.robot.commands.Intake.CoralPivotPIDCommand;
import frc.robot.commands.outake.EndEffectorPIDCommand;
import frc.robot.commands.outake.IntakeCoralEndeffector;
import frc.robot.commands.outake.PlaceCoralCommand;
import utils.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final SendableChooser<Command> autoChooser;
  
  //Original port are driverXBox = 1, driverXBox2 = 0

  public int invert = 1;
  final CommandXboxController driverXbox = new CommandXboxController(0);

  final CommandXboxController driverXbox2 = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/nemo"));
                                                                            
  
  private final ElevatorSubsytem m_ElevatorSubsytem = new ElevatorSubsytem();
  private final CoralIntakeSubsystem m_CoralIntakeSubsystem = new CoralIntakeSubsystem();
  //private final AlgaeIntakeSubsystem m_AlgaeIntakeSubsystem = new AlgaeIntakeSubsystem();
  private final EndEffectorSubsytem m_EndEffectorSubsytem = new EndEffectorSubsytem();
  private final reefposes reefpose = new reefposes();

  // Shuffle board stuff
  // private GenericEntry reefHeightTab;

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
                                                                () -> invert *driverXbox.getLeftY(),
                                                                () -> invert *driverXbox.getLeftX())
                                                            .withControllerRotationAxis(() -> -1 * driverXbox.getRightX())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(false);

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
  // Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.RIGHT_X_DEADBAND),
    () -> driverXbox.getRightX(),
    () -> driverXbox.getRightY());
    
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> driverXbox.getLeftY(),
                                                                   () -> driverXbox.getLeftX())
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
  
  // int pos = 3;
  Command placer = new PlaceCoralCommand(endEffDefaultCmd, drivebase);

  CoralIntakeWheelsCommand coralWheels = new CoralIntakeWheelsCommand(m_CoralIntakeSubsystem);

  Command CoralDrive = new ParallelCommandGroup(
    new InstantCommand(()-> m_CoralIntakeSubsystem.setPosition(IntakePosition.DRIVING)),
    coralWheels.stopMotors());

  Command CoralFloor = new SequentialCommandGroup(
    new InstantCommand(()-> m_CoralIntakeSubsystem.setPosition(IntakePosition.FLOOR)),
    new WaitUntilCommand(()->MathUtil.isNear(CoralArmConstants.coral_floorintake_pos, m_CoralIntakeSubsystem.doubleMeasurement(), 0.005)),
    coralWheels.fullIntake());

  Command HalfCoralFloor = new SequentialCommandGroup(
    new InstantCommand(()-> m_CoralIntakeSubsystem.setPosition(IntakePosition.FLOOR)),
    new WaitUntilCommand(()->MathUtil.isNear(CoralArmConstants.coral_floorintake_pos, m_CoralIntakeSubsystem.doubleMeasurement(), 0.005)),
    coralWheels.halfIntake());

  Command CoralHumanPlayer =  new SequentialCommandGroup(
    new InstantCommand(()->m_CoralIntakeSubsystem.setPosition(IntakePosition.HUMAN_STATION)),
    //new WaitUntilCommand(()->MathUtil.isNear(CoralArmConstants.coral_humanstatione_pos, m_CoralIntakeSubsystem.doubleMeasurement(), 0.005)),
    coralWheels.turnMotor(CoralArmConstants.coral_intake_humanStation_speed)
    );

  Command HalfCoralHumanPlayer =  new SequentialCommandGroup(
    new InstantCommand(()->m_CoralIntakeSubsystem.setPosition(IntakePosition.HUMAN_STATION)),
    //new WaitUntilCommand(()->MathUtil.isNear(CoralArmConstants.coral_humanstatione_pos, m_CoralIntakeSubsystem.doubleMeasurement(), 0.005)),
    coralWheels.halfIntake()
    );
  
  Command L1 = new SequentialCommandGroup(
    new InstantCommand(()-> m_CoralIntakeSubsystem.setPosition(IntakePosition.REEF)),
    new WaitUntilCommand(()->MathUtil.isNear(CoralArmConstants.coral_reef_l1, m_CoralIntakeSubsystem.doubleMeasurement(), 0.005)),
    coralWheels.turnMotor(CoralArmConstants.coral_outtake_reef_speed));

  Command HumanStationHalfIntake =  new SequentialCommandGroup(
    coralWheels.turnMotor(CoralArmConstants.coral_humanstatione_pos),
    new WaitUntilCommand(m_CoralIntakeSubsystem.hasCoral()),
    coralWheels.turnMotor(0)
    );
  // public static Level level = Level.L2;
  Command drivetotag;// = drivebase.driveToPose(new Pose2d(new Translation2d(xap,yap), new Rotation2d(rap)));

    void levelUpCommand() {
      switch (Robot.reefLevel) {
        case L2:
          Robot.reefLevel = Level.L3;
          Reef.pos = 3;
          // PlaceCoralCommand.height1 = HeightLevels.Middle_PRE;
          // PlaceCoralCommand.height2 = HeightLevels.MIDDLE;
          break;
        case L3:
          Robot.reefLevel = Level.L4;
          Reef.pos = 4;
          // PlaceCoralCommand.height1 = HeightLevels.HIGH_PRE;
          // PlaceCoralCommand.height2 = HeightLevels.HIGH;
          break;
        case L4:
          Robot.reefLevel = Level.L2;
          Reef.pos = 2;
          // PlaceCoralCommand.height1 = HeightLevels.LOW_PRE;
          // PlaceCoralCommand.height2 = HeightLevels.LOW;
          break;
      }}
  
    void levelDownCommand() {
      switch (Robot.reefLevel) {
        case L2:
          Robot.reefLevel = Level.L4;
          Reef.pos = 4;
          // PlaceCoralCommand.height1 = HeightLevels.HIGH_PRE;
          // PlaceCoralCommand.height2 = HeightLevels.HIGH;

          break;
        case L3:
          Robot.reefLevel = Level.L2;
          Reef.pos = 2;
          // PlaceCoralCommand.height1 = HeightLevels.LOW_PRE;
          // PlaceCoralCommand.height2 = HeightLevels.LOW;
          break;
        case L4:
          Robot.reefLevel = Level.L3;
          Reef.pos = 3;
          // PlaceCoralCommand.height1 = HeightLevels.Middle_PRE;
          // PlaceCoralCommand.height2 = HeightLevels.MIDDLE;
          break;
        }
      }

      Command placeAtSpot() {
        switch (SmartDashboard.getString("level", "")) {
          case "L2": return PlaceCoralCommand.l2CommandFlip();
          case "L3": return PlaceCoralCommand.l3CommandFlip();
          case "L4": return PlaceCoralCommand.l4CommandFlip();
          default: return Commands.none();
        }
      }

      public static volatile Location loc = Location.A;

      Command selectorUp = new InstantCommand(() ->{
        switch (loc) {
          case A:
          loc = Location.L;
          break;
          case B:
          loc = Location.C;
          break;
          case C:
          loc = Location.D;
          break;
          case D:
          loc = Location.E;
          break;
          case E:
          loc = Location.F;
          break;
          case F:
          loc = Location.G;
          break;
          case G:
          case H:
          break;
          case I:
          loc = Location.H;
          break;
          case J:
          loc = Location.I;
          break;
          case K:
          loc = Location.J;
          break;
          case L:
          loc = Location.K;
          break;
        }}
        );
        Command selectorDown = new InstantCommand(() ->{
          switch (loc) {
            case A:
            case B:
            break;
            case C:
            loc = Location.B;
            break;
            case D:
            loc = Location.C;
            break;
            case E:
            loc = Location.D;
            break;
            case F:
            loc = Location.E;
            break;
            case G:
            loc = Location.F;
            break;
            case H:
            loc = Location.I;
            break;
            case I:
            loc = Location.J;
            break;
            case J:
            loc = Location.K;
            break;
            case K:
            loc = Location.L;
            break;
            case L:
            loc = Location.A;
            break;
          }}
          );
          Command selectorLeft = new InstantCommand(() ->{
            switch (loc) {
              case A:
              loc = Location.L;
              break;
              case B:
              loc = Location.A;
              break;
              case C:
              loc = Location.B;
              break;
              case D:
              loc = Location.C;
              break;
              case E:
              loc = Location.F;
              break;
              case F:
              loc = Location.G;
              break;
              case G:
              loc = Location.H;
              break;
              case H:
              loc = Location.I;
              break;
              case I:
              loc = Location.J;
              break;
              case J:
              case K:
              break;
              case L:
              loc = Location.K;
              break;
            }}
            );
            Command selectorRight = new InstantCommand(() ->{
              switch (loc) {
                case A:
                loc = Location.B;
                break;
                case B:
                loc = Location.C;
                break;
                case C:
                loc = Location.D;
                break;
                case D:
                case E:
                break;
                case F:
                loc = Location.E;
                break;
                case G:
                loc = Location.F;
                break;
                case H:
                loc = Location.G;
                break;
                case I:
                loc = Location.H;
                break;
                case J:
                loc = Location.I;
                break;
                case K:
                loc = Location.L;
                break;
                                case L:
                loc = Location.A;
                break;
              }}
              );
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */  
  
  public RobotContainer() { 
    new LimeLightExtra(drivebase);

    LimelightHelpers.SetRobotOrientation(null, drivebase.getHeading().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetIMUMode(null, 4);
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("center", drivebase.centerModulesCommand().withTimeout(0.5));
    NamedCommands.registerCommand("CoralDrive", CoralDrive);
    NamedCommands.registerCommand("CoralFloor", CoralFloor);
    NamedCommands.registerCommand("CoralHumanPlayer", CoralHumanPlayer);
    NamedCommands.registerCommand("CoralL1", L1);
    NamedCommands.registerCommand("StopMotors",coralWheels.stopMotors());
    NamedCommands.registerCommand("HumanStationHalfIntake",HumanStationHalfIntake);
    

    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> false
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    System.out.println("pos" + Reef.pos
    );
   
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

      driverXbox.start().onTrue(new InstantCommand(()->invert = invert *-1));
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

      // driverXbox.b().onTrue(new InstantCommand(()->drivebase.setMaxSpeed(.5)));
      // driverXbox.b().onTrue(new InstantCommand(()->drivebase.setMaxSpeed(1)));

     // driverXbox2.right

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

      driverXbox.a().whileTrue(new InstantCommand(()-> {
        Constants.MAX_SPEED = Units.feetToMeters(14.5*0.35);
      }));

      driverXbox.a().onFalse(new InstantCommand(()-> {
        Constants.MAX_SPEED = Units.feetToMeters(14.5*Constants.SPEED_LIMITER);
      }));

      // driverXbox2.x().whileTrue(
      //   CoralHumanPlayer
      // );
      // driverXbox2.x().onFalse(
      //   CoralDrive
      // );

      driverXbox.x().onTrue(
        new SequentialCommandGroup(
        new InstantCommand(()->{
          String loca = SmartDashboard.getString("location", null);
          Boolean redAlliance = drivebase.isRedAlliance();
          if (loca==null) return ;
          double xap = reefpose.getArrayfromKey(loca, redAlliance)[0]+(redAlliance?13.058902:4.489323);
          double yap = reefpose.getArrayfromKey(loca, redAlliance)[1]+4.0259;
          double rap = reefpose.getArrayfromKey(loca, redAlliance)[2];
          drivetotag = drivebase.driveToPose(new Pose2d(new Translation2d(xap,yap), new Rotation2d(rap)));
          if (!drivetotag.isScheduled()) drivetotag.schedule();
        }
        ), 
        new InstantCommand(()->drivebase.lock())
        )
      );

      driverXbox.x().onFalse(
        new InstantCommand(()->{
          if (drivetotag.isScheduled()) drivetotag.cancel();
          }
        )
      );

      new HighTrigger(driverXbox2.getHID(), XboxController.Axis.kRightY).onTrue(selectorDown);
      new LowTrigger(driverXbox2.getHID(), XboxController.Axis.kRightY).onTrue(selectorUp);
      new LowTrigger(driverXbox2.getHID(), XboxController.Axis.kRightX).onTrue(selectorLeft);
      new HighTrigger(driverXbox2.getHID(), XboxController.Axis.kRightX).onTrue(selectorRight);

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

      // driverXbox.leftBumper().onTrue(
      //   new InstantCommand(()->currentMoveCommand = ()->drivebase.returnPose())
      //   );

      // driverXbox.leftBumper().onFalse(
      //   currentMoveCommand.get() != null ? new InstantCommand(()->currentMoveCommand.get().cancel()) : Commands.none()
      // );


      // driverXbox.leftBumper().onFalse( 
      //   ismoving ? new SequentialCommandGroup(Commands.runOnce(drivebase::lock, drivebase), new InstantCommand(()->ismoving = true)) : Commands.none()
      // );

      //driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                         );
      //driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

      // driverXbox2.a().onTrue(PlaceCoralCommand.placeAt(endEffDefaultCmd, HeightLevels.MIDDLE));

      driverXbox2.a().whileTrue(
        HalfCoralFloor
      );
      driverXbox2.a().onFalse(
        CoralDrive
      );

      driverXbox2.y().whileTrue(
        CoralFloor
      );
      driverXbox2.y().onFalse(
        CoralDrive
      );

      //Coral move to setpoint
      driverXbox2.x().whileTrue(
        new InstantCommand(() -> {
        m_CoralIntakeSubsystem.setSetpoint(.1);
        })
        );
        driverXbox2.x().onFalse(
        CoralDrive
      );

      driverXbox2.y().whileTrue(
        coralWheels.fullIntake()
      );
      driverXbox2.y().onFalse(
        CoralDrive
      );
      
      //Coral move to reef l1
      driverXbox2.b().onTrue(
        L1
      );
      driverXbox2.b().onFalse(
        CoralDrive
      );

      driverXbox2.povUp().onTrue(
        new InstantCommand(()->levelUpCommand())
      );

      driverXbox2.povDown().onTrue(
        new InstantCommand(()->levelDownCommand())
      );

      driverXbox2.leftBumper().onTrue(
        PlaceCoralCommand.resetArm()
      );
      driverXbox2.rightBumper().onTrue(
        placeAtSpot()
      );

      // sets arm to 0 pos if needed
      driverXbox2.start().onTrue(
        IntakeCoralEndeffector.quickIntakeToUp(endEffDefaultCmd)
      );

        // driverXbox2.a().onTrue(
        //   IntakeCoralEndeffector.quickIntakeFacingDown(endEffDefaultCmd)
        //   // IntakeCoralEndeffector.quickIntakeFacingDown(endEffDefaultCmd)
        // );
    }

  }

      
//       driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
