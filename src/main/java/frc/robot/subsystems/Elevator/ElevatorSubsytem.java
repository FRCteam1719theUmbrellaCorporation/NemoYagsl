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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import static edu.wpi.first.units.Units.*;

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
  private final SparkMax ELEVATOR_MOTOR_ONE;
  private final SparkMax ELEVATOR_MOTOR_TWO;
  private final RelativeEncoder ELEVATOR_ENCODER;
  //ENCODER

  static double HEIGHT_SETPOINT = 0;
  private ProfiledPIDController elevatorPIDController = new ProfiledPIDController(ElevatorConstants.ElevatorkP, ElevatorConstants.ElevatorkI, ElevatorConstants.ElevatorkD, new Constraints(ElevatorConstants.MaxVelocity, ElevatorConstants.MaxAcceleration)); // TODO ADD VALUES

  private final ElevatorFeedforward elevatorPIDfeed = new ElevatorFeedforward(ElevatorConstants.ElevatorkS, ElevatorConstants.ElevatorkG, ElevatorConstants.ElevatorkV, ElevatorConstants.ElevatorkA);

  public ElevatorSubsytem() {
    ELEVATOR_MOTOR_ONE = new SparkMax(Constants.ELEVATOR_PIN_ONE, MotorType.kBrushless);
    ELEVATOR_MOTOR_TWO = new SparkMax(Constants.ELEVATOR_PIN_TWO, MotorType.kBrushless);
    ELEVATOR_ENCODER = ELEVATOR_MOTOR_ONE.getEncoder();
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
    ELEVATOR_MOTOR_ONE.set(0.0);
    ELEVATOR_MOTOR_TWO.set(0.0);
  }

  public double doubleMeasurement() {
    return ELEVATOR_ENCODER.getPosition() * 360;
  }
  public void reachGoal(double goal){
    double voltsOutput = MathUtil.clamp(
      elevatorPIDfeed.calculateWithVelocities(ELEVATOR_ENCODER.getVelocity(), elevatorPIDController.getSetpoint().velocity) + elevatorPIDController.calculate(ELEVATOR_ENCODER.getPosition(), goal), -7,7);
      ELEVATOR_MOTOR_ONE.setVoltage(voltsOutput);
      ELEVATOR_MOTOR_TWO.setVoltage(voltsOutput);
  }

  public Command setGoal(double goal){
    return run(() -> reachGoal(goal));
  }

  public Command setElevatorHeight(double height){
    return setGoal(height).until(()->aroundHeight(height, ElevatorConstants.Tolerance));
  }
public boolean aroundHeight(double height, double tolerance){
    return MathUtil.isNear(height,ELEVATOR_ENCODER.getPosition(),tolerance);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
