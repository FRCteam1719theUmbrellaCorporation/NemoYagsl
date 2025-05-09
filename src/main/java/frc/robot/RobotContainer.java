// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoralArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Objects;

import swervelib.SwerveInputStream;
import utils.Reef.Level;
import utils.Reef.Location;
import frc.robot.subsystems.LimeLightExtra;
import frc.robot.subsystems.Elevator.ElevatorSubsytem;
import frc.robot.subsystems.Elevator.EndEffectorSubsytem;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;
import frc.robot.subsystems.intake.CoralIntakeSubsystem.IntakePosition;
import frc.robot.commands.Controls.SelectReef;
import frc.robot.commands.Controls.SelectReef;
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
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/nemo"));

  public static SelectReef reefSelector;
                                                                            
  
  private final ElevatorSubsytem m_ElevatorSubsytem = new ElevatorSubsytem();
  private final CoralIntakeSubsystem m_CoralIntakeSubsystem = new CoralIntakeSubsystem();
  //private final AlgaeIntakeSubsystem m_AlgaeIntakeSubsystem = new AlgaeIntakeSubsystem();
  private final EndEffectorSubsytem m_EndEffectorSubsytem = new EndEffectorSubsytem();
  private final reefposes reefpose = drivebase.calculatedposes;
  private final reefposes reefpose2 = new reefposes();
  private NTEpilogueBackend epilogue;

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
    coralWheels.turnMotor(CoralArmConstants.coral_intake_floor_speed));

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
    //new WaitUntilCommand(()->MathUtil.isNear(CoralArmConstants.coral_reef_l1, m_CoralIntakeSubsystem.doubleMeasurement(), 0.005)),
    coralWheels.turnMotor(CoralArmConstants.coral_outtake_reef_speed)
  );

  Command HumanStationHalfIntake =  new SequentialCommandGroup(
    new InstantCommand(()-> m_CoralIntakeSubsystem.setPosition(IntakePosition.HUMAN_STATION)),
    coralWheels.turnMotor(CoralArmConstants.coral_intake_humanStation_speed),
    new WaitUntilCommand(m_CoralIntakeSubsystem.hasCoral()),
    coralWheels.turnMotor(0)
    );

  // public static Level level = Level.L2;
  public static volatile Command drivetotag;
  public static volatile Command driveback;

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
        // System.out.println(SmartDashboard.getString("level", "L3"));
        switch (SmartDashboard.getString("level", "")) {
          // case "L2": return PlaceCoralCommand.l2CommandFlip();
          case "L3": return PlaceCoralCommand.l3CommandFlip();
          case "L4": return PlaceCoralCommand.l4CommandFlip();
          default: return Commands.none();
        }
      }

      public static volatile Location loc = Location.A;

      
      void selectorClockwiseCommand() {
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
            loc = Location.E;
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
            loc = Location.K;
            break;
          case K:
            loc = Location.L;
            break;
          case L:
            loc = Location.A;
            break;
        }
      }
      


      public void selectorCounterClockwiseCommand() {
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
            loc = Location.D;
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
            loc = Location.J;
            break;
          case L:
            loc = Location.K;
            break;
        }
      }
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */  
  
  public RobotContainer() { 
    new LimeLightExtra(drivebase);

    epilogue = new NTEpilogueBackend(NetworkTableInstance.getDefault());
    reefSelector = new SelectReef(driverXbox2::getRightX, () -> -driverXbox2.getRightY());

    LimelightHelpers.SetRobotOrientation(null, drivebase.getHeading().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetIMUMode(null, 0);
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("center", drivebase.centerModulesCommand().withTimeout(0.5));
    NamedCommands.registerCommand("CoralDrive", CoralDrive);
    NamedCommands.registerCommand("CoralFloor", CoralFloor);
    NamedCommands.registerCommand("HumanStation", CoralHumanPlayer);
    NamedCommands.registerCommand("CoralL1", L1);
    NamedCommands.registerCommand("resetarm", PlaceCoralCommand.resetArm());
    NamedCommands.registerCommand("StopMotors",coralWheels.stopMotors());
    NamedCommands.registerCommand("HumanStationHalfIntake",HumanStationHalfIntake);
    NamedCommands.registerCommand("scorel4", new SequentialCommandGroup(PlaceCoralCommand.l4CommandFlip(), PlaceCoralCommand.returnAfterPlacing()));
    NamedCommands.registerCommand("cha-chink", IntakeCoralEndeffector.quickIntakeFacingDown(endEffDefaultCmd));
    

    autoChooser = AutoBuilder.buildAutoChooser();

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
    reefpose.add(0.459502+0.02, -0.2359);

    reefpose2.add(0.459502+0.02+0.25, -0.2359);
   
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
      //driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

      driverXbox.a().whileTrue(new InstantCommand(()-> {
        Constants.MAX_SPEED = Units.feetToMeters(14.5*0.35);
      }));

      driverXbox.a().onFalse(new InstantCommand(()-> {
        Constants.MAX_SPEED = Units.feetToMeters(14.5*Constants.SPEED_LIMITER);
      }));

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
              drivetotag.schedule();
          }),

          //drivebase.MoveWithReefPose(reefpose2)
          
          new WaitUntilCommand(()->drivetotag.isFinished()),

          new InstantCommand(()->drivebase.lock()),
          new InstantCommand(()->placeAtSpot().schedule()),
          new WaitCommand(3),
          new InstantCommand(()->PlaceCoralCommand.returnAfterPlacing().schedule())
        )
          //new WaitUntilCommand(()->drivetotag.isFinished()), 
          //new WaitCommand(7),
          //new InstantCommand(()->Robot.reefLevel = Level.L4),
          //drivebase.MoveWithReefPose(reefpose)
          



          // new InstantCommand(()->{
          //   if (placeAtSpot().isFinished()) {
          //     String loca = SmartDashboard.getString("location", null);
          //     Boolean redAlliance = drivebase.isRedAlliance();
          //     if (loca==null) return ;
          //     double xap = reefpose2.getArrayfromKey(loca, redAlliance)[0]+(redAlliance?13.058902:4.489323);
          //     double yap = reefpose2.getArrayfromKey(loca, redAlliance)[1]+4.0259;
          //     double rap = reefpose2.getArrayfromKey(loca, redAlliance)[2];
          //     drivetotagback = drivebase.driveToPose(new Pose2d(new Translation2d(xap,yap), new Rotation2d(rap)));
          //     drivetotagback.schedule();
              
          //   }
          // }
          // )
        
      //   new ParallelCommandGroup(
      //     new InstantCommand(()->{
      //       String loca = SmartDashboard.getString("location", null);
      //       Boolean redAlliance = drivebase.isRedAlliance();
      //       if (loca==null) return ;
      //       double xap = reefpose.getArrayfromKey(loca, redAlliance)[0]+(redAlliance?13.058902:4.489323);
      //       double yap = reefpose.getArrayfromKey(loca, redAlliance)[1]+4.0259;
      //       double rap = reefpose.getArrayfromKey(loca, redAlliance)[2];
      //       drivetotag = drivebase.driveToPose(new Pose2d(new Translation2d(xap,yap), new Rotation2d(rap)));
      //       drivetotag.schedule();
      //   }),
      //     new WaitUntilCommand(drivebase.within()).andThen(new InstantCommand(()->placeAtSpot().schedule()))
      //   ).andThen(
      //     new SequentialCommandGroup(
      //       new WaitCommand(1),
      //       //new WaitUntilCommand(()->placeAtSpot().isFinished()),
      //       new InstantCommand(()->{
      //         String loca = SmartDashboard.getString("location", null);
      //         Boolean redAlliance = drivebase.isRedAlliance();
      //         double rap = reefpose.getArrayfromKey(loca, redAlliance)[2];
      //         driveback = drivebase.driveToPose(new Pose2d(new Translation2d(drivebase.getPose().getX()+Math.sin(rap)*0.25, drivebase.getPose().getY()+Math.cos(rap)*0.25), drivebase.getPose().getRotation()));
      //         driveback.schedule();
      //   })//,
      //       //new WaitUntilCommand(()->driveback.isFinished()).andThen(PlaceCoralCommand.returnAfterPlacing())
      //     )
      // )
      );

      driverXbox.x().onFalse(
        new InstantCommand(()->{
          if (Objects.nonNull(drivetotag) || drivetotag.isScheduled()) drivetotag.cancel();
          //if (driveback.isScheduled()) driveback.cancel();
          }
        )
      );

      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

      //Coral move to reef l1
      driverXbox2.b().whileTrue(
        L1
      );
      driverXbox2.b().onFalse(
        CoralDrive
      );

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

      driverXbox2.getRightX();

      driverXbox2.x().whileTrue(
        HumanStationHalfIntake
      );
      
      driverXbox2.x().onFalse(
        CoralDrive
      );
      

      driverXbox2.povUp().onTrue(
        new InstantCommand(()->levelUpCommand())
      );

      driverXbox2.povDown().onTrue(
        new InstantCommand(()->levelDownCommand())
      );

      driverXbox2.povRight().onTrue(
        new InstantCommand(()->selectorClockwiseCommand())
      );

      driverXbox2.povLeft().onTrue(
        new InstantCommand(()->selectorCounterClockwiseCommand())
      );

      driverXbox2.leftBumper().onTrue(
        PlaceCoralCommand.resetArm()
      );

      // sets arm to 0 pos if needed
      driverXbox2.start().onTrue(
        IntakeCoralEndeffector.quickIntakeToUp(endEffDefaultCmd)
      );


        driverXbox2.rightBumper().onTrue(
          new InstantCommand(() -> {
            PlaceCoralCommand.manualPlacement().schedule();
          })
        );

        driverXbox2.leftTrigger().onTrue( 
          IntakeCoralEndeffector.quickIntakeFacingDown(endEffDefaultCmd)
        ); 

        // hits the highest algae 
        driverXbox2.rightTrigger().onTrue( 
          PlaceCoralCommand.algaeHitter()
        ); 
    }

  }

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

  public void publishVisuals() {
    // Needs elevator distance constant
    double elevatorBaseHeight = m_ElevatorSubsytem.doubleMeasurement() * 0.0096;

    // Needs intake angle offset constant
    double intakeAngle = (m_CoralIntakeSubsystem.doubleMeasurement()-0.113) * Math.PI*2;

    // May need offset constant
    double endEffectorAngle = m_EndEffectorSubsytem.doubleMeasurement() * Math.PI*2;

    // Constants based on subsystem positioning and robot dimensions from CAD
      //epilogue.log("visuals/aprilstags", LimelightHelpers);
      
      epilogue.log("visuals/internalpose", new Pose3d[] {
        new Pose3d(0.184150, -0.295, 0.247650, new Rotation3d(0, intakeAngle, 0)),
        new Pose3d(0, 0, elevatorBaseHeight, new Rotation3d(0, 0, 0)),
        new Pose3d(0, 0, elevatorBaseHeight * 2, new Rotation3d(0, 0, 0)),
        new Pose3d(0, 0, elevatorBaseHeight * 3, new Rotation3d(0, 0, 0)),
        new Pose3d(0.038092, 0, 0.273050 + elevatorBaseHeight * 3, new Rotation3d(endEffectorAngle, 0, 0))
    }, Pose3d.struct);
  }
}
