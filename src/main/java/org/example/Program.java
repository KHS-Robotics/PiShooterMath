package org.example;

import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;

import java.io.IOException;

public class Program {
  public static void main(String[] args) throws IOException {
    NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
    WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
    WPIMathJNI.Helper.setExtractOnStaticLoad(false);
    CameraServerJNI.Helper.setExtractOnStaticLoad(false);

    CombinedRuntimeLoader.loadLibraries(Program.class, "wpiutiljni", "wpimathjni", "ntcorejni");
    new Program().run();
  }

  public void run() {
    double[] targetPose = new double[3];
    double[] robotPose = new double[5];
    double[] optimalParams = new double[3];
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("SmartDashboard");

    DoubleArraySubscriber targetSubscriber = table.getDoubleArrayTopic("targetPose").subscribe(targetPose);
    // {x, y, z}
    DoubleArraySubscriber poseSubscriber = table.getDoubleArrayTopic("robotPose").subscribe(robotPose);
    // {x, y, vx, vy, theta}
    DoubleArrayPublisher optimalParamsPublisher = table.getDoubleArrayTopic("optimalParams").publish();
    // {theta, phi, t}
    BooleanPublisher goodTrajectoryPublisher = table.getBooleanTopic("goodTrajectory").publish();


    inst.startClient4("orangepi5");
    inst.setServer("10.43.42.2"); // where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
    //inst.startDSClient(); // recommended if running on DS computer; this gets the robot IP from the DS
    while (true) {
      try {
        Thread.sleep(20);
      } catch (InterruptedException ex) {
        System.out.println("interrupted");
        return;
      }
      boolean goodTrajectory = true;
      targetPose = targetSubscriber.get();
      robotPose = poseSubscriber.get(robotPose);
      optimalParams = Shooter.optimizeShooterOrientation(1, 0.1, 0.1, targetPose[0], targetPose[1], targetPose[2], robotPose);

      // center of the exit of the shooter (middle of shooter)
      Translation3d shooterPoseRobotRelative = Shooter.shooterExitRobotRelative(optimalParams[0]);
      Translation3d shooterPose = Shooter.shooterExitFieldRelative(new Pose2d(robotPose[0], robotPose[1], Rotation2d.fromRadians(robotPose[4])), shooterPoseRobotRelative);
      double shooterAngle = optimalParams[0] - Math.toRadians(135);

      double[] in = {shooterPose.getX(), shooterPose.getY(), shooterPose.getZ(),
              robotPose[2] + (Shooter.v0 * Math.sin(Math.PI / 2 - shooterAngle) * Math.cos(optimalParams[1])),
              robotPose[3] + (Shooter.v0 * Math.sin(Math.PI / 2 - shooterAngle) * Math.sin(optimalParams[1])),
              Shooter.v0 * Math.cos(Math.PI / 2 - shooterAngle)};

      // we may not want to actually do any of this if it can't run in real time
      double[][] trajectory = Shooter.propagateWholeTrajectory3d(in, optimalParams[2], 1);
      double[] finalPoint = trajectory[trajectory.length - 1];

      goodTrajectory = !(finalPoint[5] < 0) && !(Math.abs(finalPoint[2] - targetPose[2]) > 0.2);

//      optimalParams[0] = normalizeRadians(optimalParams[0]);
//      optimalParams[1] = normalizeRadians(optimalParams[1]);

      goodTrajectoryPublisher.set(goodTrajectory);
      optimalParamsPublisher.set(optimalParams);

      //System.out.println("theta " + optimalParams[0] + " phi " + optimalParams[1] + " t " + optimalParams[2] + " finalx " + finalPoint[0] + " finaly " + finalPoint[1] + " finalz " + finalPoint[2]);


    }
  }

  public static double normalizeRadians(double angle) {
    angle = Math.toDegrees(angle);
    if (angle > 0) {
      angle %= 360;
      if (angle > 180) {
        angle -= 360;
      }
    } else if (angle < 0) {
      angle %= -360;
      if (angle < -180) {
        angle += 360;
      }
    }
    return Math.toRadians(angle);
  }
}