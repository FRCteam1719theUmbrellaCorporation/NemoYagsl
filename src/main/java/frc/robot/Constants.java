// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import static edu.wpi.first.units.Units.*;

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
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double SPEED_LIMITER = .7; // was 0.25 3/5/25
  public static final double slow_speed = .5;
  public static final double MAX_SPEED  = Units.feetToMeters(14.5*SPEED_LIMITER);

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


/*
 * CAN IDS
 */

//ANGLE MOTORS ARE RESONSIBLE FOR CHANING THE ANGLE OF THE ARM
//WHEEL SPIN IS RESPONSIBLE FOR THE INTAKE OUTAKE OF THE WHEELS 

  // public static final int ALGAE_ARM_WHEEL_SPIN_ID = 8;
  // public static final int ALGAE_ARM_ANGLE_MOTOR_ID = 7; 

  //CORAL
  public static final int CORAL_ARM_WHEEL_SPIN_ID = 23;
  public static final int CORAL_ARM_ANGLE_MOTOR_ID = 9; 
  public static final int CORAL_ARM_RANGE_SENSOR = 13;

  // ELEVATOR
  public static final int ELEVATOR_PIN_ONE = 6;
  public static final int ELEVATOR_ROTATE_ENCODER = 4;

  //ENDEFECTOR
  public static final int ENDEFECTOR_ANGLE_MOTOR_ID = 24;

  public static final double reefLength = Units.inchesToMeters(65.49);
  public static final double reefLevelDistance = Units.inchesToMeters(12.94);

  public static class IntakeDetails {
    public static final double intakePos = 0; // TODO FIX This will be the position of intaking!
  }

  public static class CoralArmConstants{
    public static final double CoralArm_kP = 1.5;
    public static final double CoralArm_kI = 0;
    public static final double CoralArm_kD = 0;

    //Imb- dont know if we need both of these
    public static final double CoralArm_startPos = 0;
    public static final double CoralArm_startAngle = 0;

    public static final double CoralArmWheelMaxVelocity = Meters.of(0).per(Second).in(MetersPerSecond);
    public static final double CoralArmWheelDefaultSpeed = Meters.of(0).per(Second).in(MetersPerSecond);

    //Imb - dont know if these are necessary
    public static final double CoralArmWheelRadius = Units.inchesToMeters(0);


    public static final double MAXROTATE = .385;
    public static final double MINROTATE = 0.09;


    public static final double ARM_OUTAKE_POS_ANGLE = 0.030;

    public static final double MIN_SPEED = -.5f;
    public static final double MAX_SPEED = .5f;

    public static final double coral_floorintake_pos = 0.36;
    public static final double coral_armdriving_pos = 0.15;
    public static final double coral_humanstatione_pos = 0.10;
    public static final double coral_reef_l1 = 0.2;

    public static final double coral_intake_floor_speed = 0.5f;
    public static final double coral_intake_floor_speed_limited = 0.2f;
    public static final double coral_intake_humanStation_speed = 0.1f;
    public static final double coral_outtake_reef_speed = -0.2f;

  }

  public static class EndefectorConstants{
    public static final double Endefector_kP = 5;
    public static final double Endefector_kI = 0;
    public static final double Endefector_kD = 0.5;

    //Imb- dont know if we need both of these
    public static final double Endefector_startPos = 0;
    public static final double Endefector_startAngle = 0;
    public static final double MIN_SPEED = -.5f;
    public static final double MAX_SPEED = .5f;

    // if the height is LESS than 48, the arm should NOT move to / from it's intake position
    public static final double INTAKE_POS_ELEVATORPOS_MAX = 48.f;

  }  

public static class ElevatorConstants{

   public static final double ElevatorkP = 0.2;
    public static final double ElevatorkI = 0;
    public static final double ElevatorkD = 0.008;
    public static final double MaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static final double MaxAcceleration = Meters.of(6).per(Second).per(Second).in(MetersPerSecondPerSecond);
    public static final double ElevatorkS = 0;
    public static final double ElevatorkG = 0;
    public static final double ElevatorkV = 0;
    public static final double ElevatorkA = 0;
    public static final double Tolerance = 0; //CHANGE

    public static final double ELEVATOR_ROOM_MAX = 84;

    public static final double MIN_SPEED = -.35f;
    public static final double MAX_SPEED = .35f;
  }

  public static double CORALPIVITMAXAMOUNT = 235;
}
