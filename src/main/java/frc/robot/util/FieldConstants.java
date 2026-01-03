package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.875958);
  public static final double fieldWidth = Units.inchesToMeters(317.000000);

  public static final Pose3d[] coralLevel1 = getReefLevel(0.72, 0.74, -33);
  public static final Pose3d[] coralLevel2 = getReefLevel(1.15, 0.74, -33);
  public static final Pose3d[] coralLevel3 = getReefLevel(1.7, 0.74, -90);

  public static final Pose3d[] scoringZone = getReefLevel(0, 2, -33);

  // algorhithm to take middle of reef and tell the code
  // where to put it depending on i (iterations)
  // first coral/0 is at (x:4.06,y: 4.58)
  // goes left if i (iterations) is even, right if i is odd (clockwise)

  private static Pose3d[] getReefLevel(double height, double radius, double degrees) {
    Pose3d[] coralsRotation = new Pose3d[12];
    for (int i = 0; i < coralsRotation.length; i++) {
      coralsRotation[i] =
          new Pose3d(
              new Translation3d((i % 2 == 0 ? -1 : 1) * Units.inchesToMeters(6.69), radius, 0)
                  .rotateBy(new Rotation3d(0, 0, (i / 2) * (Math.PI / 3) + Math.PI / 6))
                  .plus(new Translation3d(4.457556, 3.959665, height)),
              new Rotation3d(
                  0,
                  Units.degreesToRadians(degrees),
                  Math.PI / 6 + Math.PI / 2 + ((i) / 2) * (Math.PI / 3)));
    }
    return coralsRotation;
  }
}
