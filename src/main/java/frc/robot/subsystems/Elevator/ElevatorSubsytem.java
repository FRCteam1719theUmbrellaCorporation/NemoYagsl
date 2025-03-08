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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsytem extends SubsystemBase {

    //TODO: Replace with encoder positions
    //Constant list of heights represented by english. YAY
    public enum HeightLevels {
        ZERO(5, 0), // Sets to the bottom
        INTAKE(20.75, .5), //TODO: fix :( 20.75 IS INTAKE
        INTAKE_PRE_DOWN(60, .47), //TODO: fix :( 20.75 IS INTAKE
        INTAKE_UP(60, 0),
        // REEFBASE(1, 0),
        LOW_PRE(8, 0), // Sets to the lowest 
        LOW(8, .2), // Sets to the lowest 
        Middle_PRE(27.5, 0),
        MIDDLE(27.75, 0.2), // 
        HIGH_PRE(54,0),
        HIGH(54, 0),
        MAX(ElevatorConstants.ELEVATOR_ROOM_MAX, 0); // If our elevator goes higher than the third stalk, this would allow us control. maybe we shouldnt use it 

        private final double value; // value held by each enum val
        private final double armSetpoints; // value held by each enum val

        HeightLevels(double value, double arm) {
            this.value = value;
            this.armSetpoints = arm;
        }

        // returns num val
        public double numVal() {
            return value;
        }

        // returns num val
        public double armVal() {
          return armSetpoints;
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
  // private int elevatorExtraPowerTimer;
  //ENCODER

  static double HEIGHT_SETPOINT = 30;
  // private boolean temp = true;
  private static final PIDController elevatorPIDController = new PIDController(ElevatorConstants.ElevatorkP, ElevatorConstants.ElevatorkI, ElevatorConstants.ElevatorkD); // TODO ADD VALUES
  
  public ElevatorSubsytem() {
    // temp = true;
    ELEVATOR_MOTOR_ONE = new SparkMax(Constants.ELEVATOR_PIN_ONE, MotorType.kBrushless);
    ELEVATOR_ENCODER = ELEVATOR_MOTOR_ONE.getEncoder();

    currentPosEnum = HeightLevels.ZERO;
    HEIGHT_SETPOINT = 0;
    elevatorPIDController.setSetpoint(HEIGHT_SETPOINT);

    elevatorPIDController.setTolerance(.5);
    // elevatorExtraPowerTimer = 0;
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

  public double getSetPoint() {
    return HEIGHT_SETPOINT;
  }

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
    } else {
      // System.out.println("erm its out of bounds " + HEIGHT_SETPOINT);
      output = 0;
    }

    // System.out.println("ELEVATOR DISABLED. FIX THAT, OR COMMENT OUT THE ELEVATOR");
    ELEVATOR_MOTOR_ONE.set(output);
  }

}
