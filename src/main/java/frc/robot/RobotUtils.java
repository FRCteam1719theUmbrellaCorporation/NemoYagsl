package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ElevatorConstants;
import static edu.wpi.first.units.Units.*;

public class RobotUtils {
    public static class Elevator{

    public static Distance convertRotationsToDistance(Angle rotations){
      return Meters.of(rotations.in(Rotations) *
              (2 * Math.PI * ElevatorConstants.DrumRadius) / ElevatorConstants.ElevatorGearing);
    }

    public static Angle convertDistanceToRotations(Distance distance){
      return Rotations.of(distance.in(Meters) /
              (2 * Math.PI * ElevatorConstants.DrumRadius) * ElevatorConstants.ElevatorGearing);
    }
  }
}
