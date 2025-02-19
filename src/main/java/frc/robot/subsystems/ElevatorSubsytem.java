// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.security.PublicKey;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsytem extends SubsystemBase {

    //TODO: Replace with encoder positions
    //Constant list of heights represented by english. YAY
    enum HeightLevels {
        ZERO(0), // Sets to the bottom
        REEFBASE(1),
        LOW(2), // Sets to the lowest 
        MIDDLE(3), // 
        HIGH(4),
        MAX(5); // If our elevator goes higher than the third stalk, this would allow us control. maybe we shouldnt use it 

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
  private final SparkMax ELEVATOR_MOTOR;

  static double HEIGHT_SETPOINT = 0;
  private PIDController elevatorPIDController = new PIDController(ElevatorConstants.ElevatorkP, ElevatorConstants.ElevatorkI, ElevatorConstants.ElevatorkD); // TODO ADD VALUES

  public ElevatorSubsytem() {
    ELEVATOR_MOTOR = new SparkMax(Constants.ELEVATOR_PIN_ONE, MotorType.kBrushless);
  }

  // sets the pos based off an enum value
  public void setHeightWithEnum(HeightLevels pos) {
    HEIGHT_SETPOINT = pos.numVal();
  }

  // returns the height double as an enum. easier to read
  public HeightLevels getHeightAsEnum() {
    return HeightLevels.doubleToEnum(HEIGHT_SETPOINT);
  }

  // increments up stages. this will probably be used 
  public void incrementByStage(int change) throws IndexOutOfBoundsException{
    if (Math.abs(change) > 5) throw new IndexOutOfBoundsException();
    HeightLevels[] vals = HeightLevels.values();
    // int currentArrPos = -1;

    for (int i = 0; i < vals.length; i++) {
        if (vals[i].numVal() == HEIGHT_SETPOINT) {
            try {
                HEIGHT_SETPOINT = vals[i + change].numVal();
                return;
            } catch (Exception e) {
                throw new IndexOutOfBoundsException();
            }
        }
    }
    
  }

  // controls height with double; not highly recommended
  public void controlWithDouble(double setpoint) {
    HEIGHT_SETPOINT = setpoint;
  }

  public void zeroElevator() {
    setHeightWithEnum(HeightLevels.ZERO);;
  }

  public void stop()    {
    ELEVATOR_MOTOR.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
