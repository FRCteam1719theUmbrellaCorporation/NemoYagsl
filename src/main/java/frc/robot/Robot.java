// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LimeLightExtra;
import swervelib.imu.Pigeon2Swerve;
import swervelib.imu.SwerveIMU;
import utils.Reef;
// import edu.wpi.first.networktables.DoubleSubscriber;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import utils.Reef.Level;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{
  public static volatile Reef.Level reefLevel = Reef.Level.L2;


  public static ShuffleboardTab reefTab;
  // private GenericEntry reefHeightTab;
  // private GenericEntry reefSideTab;

  private static Robot   instance;
  private        Command m_autonomousCommand;
  // public static DoubleSubscriber posSetter;
  // public static DoubleSubscriber posGetter;

  
  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  BooleanLogEntry myBooleanLog;
  DoubleLogEntry myDoubleLog;
  StringLogEntry myStringLog;

  public static boolean inAuto;

  public Robot()
  {
    instance = this;
    DataLogManager.start();

    DriverStation.startDataLog(DataLogManager.getLog());
    DriverStation.startDataLog(DataLogManager.getLog(), true);

// <<<<<<< HEAD

// =======
//     reefTab = Shuffleboard.getTab("ReefSelector");
//     reefHeightTab = Robot.reefTab.add("level", Robot.reefLevel.toString()).getEntry();
//     reefSideTab = Robot.reefTab.add("side", RobotContainer.loc.toString()).getEntry();
// >>>>>>> ccb7f8e3ae51dd538c4812497e7c64d1dcde1923
    //DataLog log = DataLogManager.getLog();
    // myBooleanLog = new BooleanLogEntry(log, "/my/boolean");
    // myDoubleLog = new DoubleLogEntry(log, "/my/double");
    // myStringLog = new StringLogEntry(log, "/my/string");
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //gyrogyro = m_robotContainer.drivebase.getSwerveDrive().getGyro();
    //m_robotContainer.drivebase.newzeroGyro();

    m_robotContainer.drivebase.zeroGyroWithAlliance();
    LimelightHelpers.SetIMUAssistAlpha(null, 0.01);


    
    //m_robotContainer.drivebase.getSwerveDrive().swerveController.lastAngleScalar = 0;

    //m_robotContainer.drivebase.zeroGyroWithAlliance();
    

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putString("level", reefLevel.toString());
    SmartDashboard.putString("location", RobotContainer.loc.toString());
    //System.out.println(SmartDashboard.getString("level", "L3"));

    // Send visual data to Networktables for AdvantageScope
    m_robotContainer.publishVisuals();
    
    if (m_robotContainer.drivebase.isRedAlliance()) LimelightHelpers.SetRobotOrientation(null, 
      Units.radiansToDegrees(m_robotContainer.drivebase.getSwerveDrive().getGyro().getRotation3d().getZ()),
      m_robotContainer.drivebase.getSwerveDrive().getGyro().getYawAngularVelocity().in(DegreesPerSecond),
      0.0,0.0,0.0,0.0
    ); 
    else LimelightHelpers.SetRobotOrientation(null, 
    Units.radiansToDegrees(m_robotContainer.drivebase.getSwerveDrive().getGyro().getRotation3d().plus(new Rotation3d(0,0,Math.PI)).getZ()),
    m_robotContainer.drivebase.getSwerveDrive().getGyro().getYawAngularVelocity().in(DegreesPerSecond),
    0.0,0.0,0.0,0.0
    );

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    LimelightHelpers.SetIMUMode(null, 0);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    inAuto = true;
    try {
      LimelightHelpers.SetIMUMode(null, 0);
      LimelightHelpers.SetIMUAssistAlpha(null, 0.01);
      LimelightHelpers.SetFiducialIDFiltersOverride(null, new int[] {100});
    } catch (Exception e) {
      System.out.println("ll did not init. terror");
    }
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_robotContainer.drivebase.zeroGyroWithAlliance();

    //m_robotContainer.drivebase.zeroGyroWithAlliance();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
    

  }

  @Override
  public void teleopInit()
  {
    inAuto = false;
    // This makes sure t\hat the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    LimelightHelpers.SetIMUMode(null, 0);
    LimelightHelpers.SetFiducialIDFiltersOverride(null, new int[] {6,7,8,9,10,11,17,18,19,20,21,22});
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    RobotContainer.reefSelector.execute();
    //myBooleanLog.append(); 
    //myDoubleLog.append(3);


  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    LimelightHelpers.SetIMUMode(null, 0);
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
