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

    //Constant list of heights represented by english. YAY
    //Side note: this is gods ugliested enum.. :(
    public enum HeightLevels {
        ZERO(5, 0.0), // Sets to the bottom
        INTAKE(22.9, .5), 
        INTAKE_WITH_ARN_DOWN(30, .5), // this sets the arm hovering above the coral
        INTAKE_PRE_DOWN(60, .5),
        INTAKE_UP(60, 0.03),
        INTAKE_FLIP_AROUND(ElevatorConstants.ARM_180_SPIN, 0, false),
        INTAKE_FLIP_TO_DOWN(ElevatorConstants.ARM_180_SPIN+3, 0.5, true),

        // REEFBASE(1, 0),
        LOW_PRE(8, 0.0), // Sets to the lowest 
        LOW(10f, .8), // Sets to the lowest 
        Middle_PRE(30.5, 0.0),
        MIDDLE(30.75, 0.8), // 
        HIGH_PRE(58,0.0),
        HIGH(58, 0.8, false),
        MAX(ElevatorConstants.ELEVATOR_ROOM_MAX, 0); // If our elevator goes higher than the third stalk, this would allow us control. maybe we shouldnt use it 

        private final double value; // value held by each enum val
        private final double armSetpoints; // value held by each enum val
        private final Boolean direction; // value held by each enum val


        HeightLevels(double value, double arm) {
            this.value = value;
            this.armSetpoints = arm;
            this.direction = null;
        }

        HeightLevels(double value, double arm, Boolean explicitDirection) {
          this.value = value;
          this.armSetpoints = arm;
          this.direction = explicitDirection;
      }

        /**
         * This is the setpoint associated with the elevator
         * 
         * @return elevator desired position
         */
        public double numVal() {
            return value;
        }

        /**
         * This is the setpoint associated with the endeffecto
         * 
         * @return endeffector desired position
         */
        public double armVal() {
          return armSetpoints;
        }

        /**
         * Gets prefered direction of the endeffector in this position
         * 
         * @return Boolean associated with it's prefered direction
         */
        public Boolean preferedDirection() {
          return direction;
        }

        /**
         * turns a double associated with the elevator into an enum
         * 
         * 
         * @param input: a double you'd like to attempt to cast into an enum
         * @return a enum if the double is valid, or an enum val
         */
        public static HeightLevels doubleToEnum(double input) {
            for (HeightLevels h: HeightLevels.values()) {
                if (h.numVal() == input) return h;
            }

            return null; // TODO: Exception?
        }

    }

 /**
  * Hardware:
  */
  private final SparkMax ELEVATOR_MOTOR_ONE;
  private final RelativeEncoder ELEVATOR_ENCODER;

  /**
   * Setpoint stuff
   */
  static double HEIGHT_SETPOINT = 30;
  private HeightLevels currentPosEnum;

  private static final PIDController elevatorPIDController = new PIDController(ElevatorConstants.ElevatorkP, ElevatorConstants.ElevatorkI, ElevatorConstants.ElevatorkD); // TODO ADD VALUES
  
  public ElevatorSubsytem() {
    //defines motors
    ELEVATOR_MOTOR_ONE = new SparkMax(Constants.ELEVATOR_PIN_ONE, MotorType.kBrushless);
    ELEVATOR_ENCODER = ELEVATOR_MOTOR_ONE.getEncoder();

    // setpoints
    currentPosEnum = HeightLevels.ZERO;
    HEIGHT_SETPOINT = 0;
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

  /**
   * Sets the elevators setpoint
   * 
   * @param setSetpoint: sets the elevators setpoint to the specified double
   */
  public void setSetpoint(double setSetpoint) {
    HEIGHT_SETPOINT = setSetpoint;
    elevatorPIDController.setSetpoint(setSetpoint);
  }

  /**
   * sends the elevator back to 0
   */
  public void zeroElevator() {
    setHeightWithEnum(HeightLevels.ZERO);;
  }

  /**
   * stops the elevator
   */
  public void stop()    {
    ELEVATOR_MOTOR_ONE.set(0.0);
  }

  /**
   * Returns the elevators current position, which should range from 0 - 84.
   * if you get a number out of those bounds, something is probably horribly wrong
   */
  public double doubleMeasurement() {
    return ELEVATOR_ENCODER.getPosition();
  }

  /**
   * Returns the current height level stored in the elevator
   */
  public HeightLevels currentPos() {
    return this.currentPosEnum;
  }

  /**
   * Returns the current set point stored in the elevator
   */
  public double getSetPoint() {
    return HEIGHT_SETPOINT;
  }

  /**
   * Checks if the elevator is around a certain height
   * 
   * @param height height to compare to
   * @param tolerance tne amount of tolerance to check
   * @return if the elevator is around that point, returns true, else false
   */
  public boolean aroundHeight(double height, double tolerance){
    return MathUtil.isNear(height,ELEVATOR_ENCODER.getPosition(),tolerance);
  }

  /**
   * Returns if the elevator is within the 0 - 84 domain
   * @return true if the elevator is within the domain, false if it's not
   */
  private boolean inBounds() {
    return HEIGHT_SETPOINT > 0 && ElevatorConstants.ELEVATOR_ROOM_MAX >= HEIGHT_SETPOINT;
  }

  /**
   * Constant control of the elevator
   * This will allow the elevator to ALWAYS hold it's position
   */
  @Override
  public void periodic() {
    double output; // this is the speed that the elevator will move at

    // checks to see if the elevator setpoint is safe 
    if (inBounds()) {
      output = MathUtil.clamp(elevatorPIDController.calculate(doubleMeasurement()), ElevatorConstants.MIN_SPEED, ElevatorConstants.MAX_SPEED);
    } else {
      output = 0;
    }

    // moves the elevator at the output speed
    ELEVATOR_MOTOR_ONE.set(output);
  }
}