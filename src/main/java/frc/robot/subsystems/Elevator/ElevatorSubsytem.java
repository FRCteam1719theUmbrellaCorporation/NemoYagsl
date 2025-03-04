// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;

// import java.security.PublicKey;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.units.PerUnit;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.Command;

// import static edu.wpi.first.units.Units.*;

public class ElevatorSubsytem extends SubsystemBase {

    //TODO: Replace with encoder positions
    //Constant list of heights represented by english. YAY
    public enum HeightLevels {
        ZERO(0.1), // Sets to the bottom
        INTAKE(30),
        REEFBASE(1),
        LOW(2), // Sets to the lowest 
        MIDDLE(3), // 
        HIGH(4),
        MAX(ElevatorConstants.ELEVATOR_ROOM_MAX); // If our elevator goes higher than the third stalk, this would allow us control. maybe we shouldnt use it 

        private final double value; // value held by each enum val

        HeightLevels(double value) {
            this.value = value;
        }

        // returns num val
        public double numVal() {
            return value;
        }

        // returns an enum based off an associated input
        public static HeightLevels doubleToEnum(double input) {
            for (HeightLevels h: HeightLevels.values()) {
                if (h.numVal() == input) return h;
            }

            return null; // TODO: Exception?
        }

    }

  /** Creates a new ElevatorSubsytem. */
  private final SparkMax ELEVATOR_MOTOR_ONE;
  private final RelativeEncoder ELEVATOR_ENCODER;
  private HeightLevels currentPosEnum;
  //ENCODER

  static double HEIGHT_SETPOINT = 30;
  // private boolean temp = true;
  private static final PIDController elevatorPIDController = new PIDController(ElevatorConstants.ElevatorkP, ElevatorConstants.ElevatorkI, ElevatorConstants.ElevatorkD); // TODO ADD VALUES

  // private static final ElevatorFeedforward elevatorPIDfeed = new ElevatorFeedforward(ElevatorConstants.ElevatorkS, ElevatorConstants.ElevatorkG, ElevatorConstants.ElevatorkV, ElevatorConstants.ElevatorkA);

  public ElevatorSubsytem() {
    // temp = true;
    ELEVATOR_MOTOR_ONE = new SparkMax(Constants.ELEVATOR_PIN_ONE, MotorType.kBrushless);
    ELEVATOR_ENCODER = ELEVATOR_MOTOR_ONE.getEncoder();

    currentPosEnum = HeightLevels.ZERO;
    HEIGHT_SETPOINT = 80;
    elevatorPIDController.setSetpoint(HEIGHT_SETPOINT);

    elevatorPIDController.setTolerance(.5);
  }

  // sets the pos based off an enum value
  public void setHeightWithEnum(HeightLevels pos) {
    setSetpoint(pos.numVal());
  }

  // returns the height double as an enum. easier to read
  public static HeightLevels doubleAsEnum(double height) {
    return HeightLevels.doubleToEnum(height);
  }

  // increments up stages. this will probably be used 
  public void incrementByStage(int change) throws IndexOutOfBoundsException{
    if (Math.abs(change) > 5) throw new IndexOutOfBoundsException();
    HeightLevels[] vals = HeightLevels.values();
    // int currentArrPos = -1;

    for (int i = 0; i < vals.length; i++) {
        if (vals[i].numVal() == HEIGHT_SETPOINT) {
            try {
                setSetpoint(vals[i + change].numVal());
                return;
            } catch (Exception e) {
                throw new IndexOutOfBoundsException();
            }
        }
    }
  }

  // controls height with double; not highly recommended
  public void controlWithDouble(double setpoint) {
    if (setpoint < 0) {
      setpoint = 0;
    } else if (setpoint > ElevatorConstants.ELEVATOR_ROOM_MAX) {
      setpoint = ElevatorConstants.ELEVATOR_ROOM_MAX;
    }

    setSetpoint(setpoint);
  }

  public void setSetpoint(double setSetpoint) {
    HEIGHT_SETPOINT = setSetpoint;
    elevatorPIDController.setSetpoint(setSetpoint);
  }

  public void zeroElevator() {
    setHeightWithEnum(HeightLevels.ZERO);;
  }

  public void stop()    {
    ELEVATOR_MOTOR_ONE.set(0.0);
  }

  public double doubleMeasurement() {
    return ELEVATOR_ENCODER.getPosition();
  }

  public HeightLevels currentPos() {
    return this.currentPosEnum;
  }

  
  // public void reachGoal(double goal){
  //   double voltsOutput = MathUtil.clamp(elevatorPIDfeed.calculateWithVelocities(ELEVATOR_ENCODER.getVelocity(), elevatorPIDController.getSetpoint().velocity) + elevatorPIDController.calculate(ELEVATOR_ENCODER.getPosition(), goal), -7,7);
  //   ELEVATOR_MOTOR_ONE.setVoltage(voltsOutput);
  // }

  // public Command setGoal(double goal){
  //   return run(() -> reachGoal(goal));
  // }

  // public Command setElevatorHeight(double height){
  //   setSetpoint(height);
  // }

  public boolean aroundHeight(double height, double tolerance){
    return MathUtil.isNear(height,ELEVATOR_ENCODER.getPosition(),tolerance);
  }

  private boolean inBounds() {
    return HEIGHT_SETPOINT > 0 && ElevatorConstants.ELEVATOR_ROOM_MAX >= HEIGHT_SETPOINT;
  }

  @Override
  public void periodic() {
    double output;
    
    if (inBounds()) {
   
      output = MathUtil.clamp(elevatorPIDController.calculate(doubleMeasurement()), ElevatorConstants.MIN_SPEED, ElevatorConstants.MAX_SPEED);
      // if (doubleMeasurement() < 10) {
      //   output = output > 0 ? 1 : output;
      // }
      // System.out.println("iOutput" + output);
      System.out.println(ELEVATOR_ENCODER.getPosition());

    } else {
      output = 0;
    }

    // if (this.doubleMeasurement() < HEIGHT_SETPOINT + 30 && temp) {
    //   ELEVATOR_MOTOR_ONE.set(.3);
    //   System.out.println("output " + ELEVATOR_MOTOR_ONE.getAppliedOutput());
    // } else {
    //   temp = false;
    //   ELEVATOR_MOTOR_ONE.set(0);

    // }
    System.out.println(output);
    ELEVATOR_MOTOR_ONE.set(output);
  }

}
