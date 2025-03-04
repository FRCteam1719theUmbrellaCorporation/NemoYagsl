// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import static edu.wpi.first.units.Units.*;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.wpilibj.DigitalSource;
// import edu.wpi.first.wpilibj.DutyCycle;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double SPEED_LIMITER = 0.25;
  public static final double MAX_SPEED  = Units.feetToMeters(14.5*SPEED_LIMITER);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }
  public static final CANBus kCANBus = new CANBus("Drivetrain", "./logs/example.hoot");

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }
  

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }


  //MOST MEGA TODO
  // PROPRELY NAME ALL VARIABLES TO BE SPECIFIC TO EACH PART OF THE ROBOT
  //I.E. ENDEFECTOR_ROTATE_MOTOR_ID
  //PART OF THE ROBOT, PURPOSE, MOTOR/ENCODER, ID(IF NECESSARY)

  /*
   * ROBOT COMPONENT IDS GO HERE. MAKE SURE TO ORDER THEM BY RELEVANCE
   */
  // TODO: CHANGE THIS HERE!

//ANGLE MOTORS ARE RESONSIBLE FOR CHANING THE ANGLE OF THE ARM
//WHEEL SPIN IS RESPONSIBLE FOR THE INTAKE OUTAKE OF THE WHEELS 

  //ALGAE
  public static final int ALGAE_ARM_WHEEL_SPIN_ID = 8;
  public static final int ALGAE_ARM_ANGLE_MOTOR_ID = 7; 
  public static final int ALGAE_ARM_INTAKE_ENCODER_ANGLE_MOTOR = 1000;

  //CORAL
  public static final int CORAL_ARM_WHEEL_SPIN_ID = 23;
  public static final int CORAL_ARM_ANGLE_MOTOR_ID = 9; 

  // ELEVATOR
  public static final int ELEVATOR_PIN_ONE = 6;
  public static final int ELEVATOR_ROTATE_ENCODER = 4;

  //ENDEFECTOR
  public static final int ENDEFECTOR_ANGLE_MOTOR_ID = 24;



  // MEGA TODO: TUNE AND ADD THESE PID VALUES
  //These PIDS were created by HBG prior to 2/19/25
 // public static final double ARMANGLE_kP = 0;
  //public static final double ARMANGLE_kI = 0;
  //public static final double ARMANGLE_kD = 0;

  //public static final double DEFAULT_INTAKE_ANGLE = 0;

  

  public static class IntakeDetails {
    public static final double intakePos = 0; // TODO FIX This will be the position of intaking!
  }



  public static class CoralArmConstants{
    public static final double CoralArm_kP = 2;
    public static final double CoralArm_kI = 0;
    public static final double CoralArm_kD = 0;

    //Imb- dont know if we need both of these
    public static final double CoralArm_startPos = 0;
    public static final double CoralArm_startAngle = 0;

    public static final double CoralArmWheelMaxVelocity = Meters.of(0).per(Second).in(MetersPerSecond);
    public static final double CoralArmWheelDefaultSpeed = Meters.of(0).per(Second).in(MetersPerSecond);

    //Imb - dont know if these are necessary
    public static final double CoralArmWheelRadius = Units.inchesToMeters(0);
    public static final double MAXROTATE = .265f;
    public static final double ARM_OUTAKE_POS_ANGLE = 0.030;
    public static final double MIN_SPEED = -.5f;
    public static final double MAX_SPEED = .5f;
  }
  public static class AlgaeArmConstants{
    public static final double AlgaeArm_kP = 2;
    public static final double AlgaeArm_kI = 0;
    public static final double AlgaeArm_kD = 0;

    //Imb- dont know if we need both of these
    public static final double ALgaeArm_startPos = 0;
    public static final double AlgaeArm_startAngle = 0;

    public static final double AlgaeArmWheelMaxVelocity = Meters.of(0).per(Second).in(MetersPerSecond);
    public static final double AlgaeArmWheelDefaultSpeed = Meters.of(0).per(Second).in(MetersPerSecond);
    public static final double AlgaeArmWheelMaxAcceleration = 0;


    //Imb - dont know if these are necessary
    public static final double AlgaeArmWheelRadius = Units.inchesToMeters(0);
    // public static final double AlgaeArm_kS = 0;
    // public static final double AlgaeArm_kG = 0;
    // public static final double AlgaeArm_kV = 0;
    // public static final double AlgaeArm_kA = 0;

    public static final double MIN_SPEED = -.5f;
    public static final double MAX_SPEED = .5f;
  } 


  public static class EndefectorConstants{
    public static final double Endefector_kP = 0;
    public static final double Endefector_kI = 0;
    public static final double Endefector_kD = 0;

    //Imb- dont know if we need both of these
    public static final double Endefector_startPos = 0;
    public static final double Endefector_startAngle = 0;
    public static final double MIN_SPEED = -.5f;
    public static final double MAX_SPEED = .5f;

  }  



  public static class ElevatorConstants{
//These are all the un-tuned constants from what IMB did on 2/18/25
//These changes should all be in the hbg/Subsystem branch in Nemo YAGSL

   public static final double ElevatorkP = 0.5;
    public static final double ElevatorkI = 0;
    public static final double ElevatorkD = 0.01;
    public static final double MaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static final double MaxAcceleration = Meters.of(6).per(Second).per(Second).in(MetersPerSecondPerSecond);
    public static final double ElevatorkS = 0;
    public static final double ElevatorkG = 0;
    public static final double ElevatorkV = 0;
    public static final double ElevatorkA = 0;
    public static final double Tolerance = 0; //CHANGE

    public static final double ELEVATOR_ROOM_MAX = 84;
  /* public static final double RampRate = 5;
    public static final double ElevatorGearing = 5;
    public static final double ElevatorCarriageMass = 5;
    public static final double DrumRadius = Units.inchesToMeters(2.0);
    public static final double MinHeightMeters = 5;
    public static final double MaxHeightMeters = 5;

//idk what these are - IMB

    public static final double Length = Inches.of(33).in(Meters);
    public static final Distance StartingHeightSim = Meters.of(0.0);
    public static final Angle StartingAngle = Degrees.of(-90);
    public static final Distance LaserCANOffset  = Inches.of(3);
    public static final double DefaultTolerance = Inches.of(1).in(Meters);
    
  */
    public static final double MIN_SPEED = -.5f;
    public static final double MAX_SPEED = .5f;
  }


  public static double CORALPIVITMAXAMOUNT = 235;
}
