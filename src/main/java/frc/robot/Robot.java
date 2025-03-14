// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
  public static Reef.Level reefLevel = Reef.Level.L2;
  private static Robot   instance;
  private        Command m_autonomousCommand;
  // public static DoubleSubscriber posSetter;
  // public static DoubleSubscriber posGetter;

  
  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  private SwerveIMU gyrogyro;

  BooleanLogEntry myBooleanLog;
  DoubleLogEntry myDoubleLog;
  StringLogEntry myStringLog;

  public Robot()
  {
    instance = this;
    DataLogManager.start();

    DriverStation.startDataLog(DataLogManager.getLog());
    DriverStation.startDataLog(DataLogManager.getLog(), false);
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


    
    m_robotContainer.drivebase.getSwerveDrive().swerveController.lastAngleScalar = 0;
    

    


    

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
    SmartDashboard.putBoolean("L2", reefLevel == Reef.Level.L2);
    SmartDashboard.putBoolean("L3", reefLevel == Reef.Level.L3);
    SmartDashboard.putBoolean("L4", reefLevel == Reef.Level.L4);
    SmartDashboard.putBoolean("A", RobotContainer.loc == Reef.Location.A);
    SmartDashboard.putBoolean("B", RobotContainer.loc == Reef.Location.B);
    SmartDashboard.putBoolean("C", RobotContainer.loc == Reef.Location.C);
    SmartDashboard.putBoolean("D", RobotContainer.loc == Reef.Location.D);
    SmartDashboard.putBoolean("E", RobotContainer.loc == Reef.Location.E);
    SmartDashboard.putBoolean("F", RobotContainer.loc == Reef.Location.F);
    SmartDashboard.putBoolean("G", RobotContainer.loc == Reef.Location.G);
    SmartDashboard.putBoolean("H", RobotContainer.loc == Reef.Location.H);
    SmartDashboard.putBoolean("I", RobotContainer.loc == Reef.Location.I);
    SmartDashboard.putBoolean("J", RobotContainer.loc == Reef.Location.J);
    SmartDashboard.putBoolean("K", RobotContainer.loc == Reef.Location.K);
    SmartDashboard.putBoolean("L", RobotContainer.loc == Reef.Location.L);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
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
    try {
      LimelightHelpers.SetIMUMode(null, 2);
    } catch (Exception e) {
      System.out.println("ll did not init. terror");
    }
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_robotContainer.drivebase.newzeroGyro();

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
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    LimelightHelpers.SetIMUMode(null, 2);
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

    //myBooleanLog.append(); 
    //myDoubleLog.append(3);


  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    LimelightHelpers.SetIMUMode(null, 2);
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
