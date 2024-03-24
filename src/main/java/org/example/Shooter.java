package org.example;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.optim.*;
import org.apache.commons.math3.optim.linear.NonNegativeConstraint;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.BOBYQAOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.MultiDirectionalSimplex;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.SimplexOptimizer;

import java.util.function.Function;

public class Shooter {
  static Function<double[], double[]> projectileEquation3d;

  private static final double DRAG_COEFFICIENT = 0.5;
  private static final double AIR_DENSITY = 1.225;
  private static final double CROSS_SECTIONAL_AREA = 0.018;
  private static final double NOTE_MASS = 0.2353;
  private static final double MU = (DRAG_COEFFICIENT * AIR_DENSITY * CROSS_SECTIONAL_AREA) / (2 * NOTE_MASS);

  private static final double SHOOTER_PIVOT_TO_END = 0.37516;
  private static final Translation3d SHOOTER_PIVOT_ROBOT_REL = new Translation3d(-0.2757, 0, 0.5972);
  private static final Translation3d ARM_PIVOT_ROBOT_REL = new Translation3d(Units.inchesToMeters(-6), 0, Units.inchesToMeters(21.5));
  private static final double ARM_LENGTH = Units.inchesToMeters(17);
  public static final double v0 = 15;

  public Shooter() {

  }

  static {
    projectileEquation3d = (double[] x) -> {
      double vx = x[3];
      double vy = x[4];
      double vz = x[5];
      double v = Math.sqrt((vx * vx) + (vy * vy) + (vz * vz));
      double ax = -MU * vx * v;
      double ay = -MU * vy * v;
      double az = -9.8 - (MU * vz * v);

      return new double[]{vx, vy, vz, ax, ay, az, 0, 0};
    };
  }


  public static double[] rkFour(double[] x, Function<double[], double[]> f) {
    double h = x[x.length - 1];

    double[] k_1 = f.apply(x);
    double[] k_2 = f.apply(addVectors(x, multiplyByScalar(k_1, h / 2)));
    double[] k_3 = f.apply(addVectors(x, multiplyByScalar(k_2, h / 2)));
    double[] k_4 = f.apply(addVectors(x, multiplyByScalar(k_3, h)));

    double[] out = addVectors(multiplyByScalar(addVectors(addVectors(addVectors(k_1, multiplyByScalar(k_2, 2)), multiplyByScalar(k_3, 2)), k_4), h / 6), x);
    out[x.length - 2] += h;
    return out;
  }

  public static double[] addVectors(double[] a, double[] b) {
    double[] out = new double[a.length];
    for (int i = 0; i < a.length; i++) {
      out[i] = a[i] + b[i];
    }
    return out;
  }

  public static double[] multiplyByScalar(double[] a, double b) {
    double[] out = new double[8];
    for (int i = 0; i < a.length; i++) {
      out[i] = a[i] * b;
    }
    return out;
  }

  public static double[][] propagateWholeTrajectory3d(double[] k, double t, int intervals) {
    double x = k[0];
    double y = k[1];
    double z = k[2];
    double vx = k[3];
    double vy = k[4];
    double vz = k[5];

    double[][] out = new double[intervals][k.length + 2];
    double dt = t / intervals;

    double[] state = {x, y, z, vx, vy, vz, 0, dt};

    for (int i = 0; i < intervals; i++) {
      state = rkFour(state, projectileEquation3d);

      out[i] = state;
    }

    return out;
  }

  public static Translation3d shooterExitRobotRelative(double theta) {
    double groundRelativeAngle = (theta - Math.toRadians(135));
    double tx = SHOOTER_PIVOT_TO_END * Math.cos(groundRelativeAngle);
    double ty = 0;
    double tz = SHOOTER_PIVOT_TO_END * Math.sin(groundRelativeAngle);

    return shooterPivotRobotRelative(theta).plus(new Translation3d(tx, ty, tz));
  }

  public static Translation3d shooterPivotRobotRelative(double theta) {
    // theta is arm angle radians
    double tx = ARM_LENGTH * Math.cos(theta);
    double ty = 0;
    double tz = ARM_LENGTH * Math.sin(theta);

    return ARM_PIVOT_ROBOT_REL.plus(new Translation3d(tx, ty, tz));
  }

  public static Translation3d shooterExitFieldRelative(Pose2d robotPose, Translation3d shooterRobotRelative) {
    double diffX = shooterRobotRelative.getX();

    Translation3d shooterFieldRelative = new Translation3d(
            robotPose.getX() + (diffX * robotPose.getRotation().getCos()),
            robotPose.getY() + (diffX * robotPose.getRotation().getSin()), shooterRobotRelative.getZ());
    return shooterFieldRelative;
  }

  public static double[] optimizeShooterOrientation(double initialTheta, double initialPhi, double initialTime,
                                                    double targetX, double targetY, double targetZ, double[] robotPose) {

    MultivariateFunction f = point -> {
      // calculate trajectory given theta phi and t
      Pose2d pose = new Pose2d(robotPose[0], robotPose[1], Rotation2d.fromRadians(robotPose[4]));
      Translation3d shooterPose = shooterExitFieldRelative(pose, shooterExitRobotRelative(point[0]));
      double shooterAngle = (point[0] - Math.toRadians(135));

      double[] in = { shooterPose.getX(), shooterPose.getY(), shooterPose.getZ(),
              robotPose[2] + (v0 * Math.sin(Math.PI / 2 - shooterAngle) * Math.cos(point[1])),
              robotPose[3] + (v0 * Math.sin(Math.PI / 2 - shooterAngle) * Math.sin(point[1])),
              v0 * Math.cos(Math.PI / 2 - shooterAngle) };
      double[][] trajectory = propagateWholeTrajectory3d(in, point[2], 1);
      double[] finalPosition = trajectory[trajectory.length - 1];

      double xdiff = targetX - finalPosition[0];
      double ydiff = targetY - finalPosition[1];
      double zdiff = targetZ - finalPosition[2];

      double distance = Math.sqrt((xdiff * xdiff) + (ydiff * ydiff) + (zdiff * zdiff));
//            System.out.println("x " + finalPosition[0] + " y " + finalPosition[1] + " z " + finalPosition[2] + " dist " + distance);
//            System.out.println("theta " + point[0] + " phi " + point[1] + " t " + point[2]);

      return distance;
    };

    ObjectiveFunction objective = new ObjectiveFunction(f);

    double[] initialGuess = new double[] { initialTheta, initialPhi, initialTime };
    InitialGuess guess = new InitialGuess(initialGuess);
    MaxIter maxIter = new MaxIter(20000);
    // look at my lawyer dawg I'm goin to jail!!!
    MaxEval maxEval = new MaxEval(100000);
    MultiDirectionalSimplex simplex = new MultiDirectionalSimplex(3, 0.001);
    SimplexOptimizer optimizer = new SimplexOptimizer(new SimpleValueChecker(0.00000001, 0.00000001));

    BOBYQAOptimizer optimizer1 = new BOBYQAOptimizer(7, 0.0000001, 0.0000001);

    NonNegativeConstraint constraint = new NonNegativeConstraint(true);
    SimpleBounds bounds = new SimpleBounds(new double[] {0, 0, 0.001}, new double[] {Math.toRadians(360 * 0.81), 2 * Math.PI, 0.5});

    PointValuePair optimum = optimizer1.optimize(
            maxIter,
            maxEval,
            objective,
            GoalType.MINIMIZE,
            guess,
            //simplex,
            constraint,
            bounds

    );

    return optimum.getPoint();
  }
}
