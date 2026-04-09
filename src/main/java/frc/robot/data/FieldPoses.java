package frc.robot.data;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;

public class FieldPoses {
    public static double getDistanceToHub(Pose2d pose) {
        Pose2d hub = getAllianceHubPose2d();
        return Math.hypot(pose.getX() - hub.getX(), pose.getY() - hub.getY());
    }

    public static Pose2d getAllianceHubPose2d() {
        return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red) ? FieldConstants.kRedHubPose
                : FieldConstants.kBlueHubPose;
    }

    public static Pose2d getAllianceLeftPassingPoint() {
        return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)
                ? FieldConstants.kPassingPointRedLeftPose
                : FieldConstants.kPassingPointBlueLeftPose;
    }

    public static Pose2d getAllianceRightPassingPoint() {
        return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)
                ? FieldConstants.kPassingPointRedRightPose
                : FieldConstants.kPassingPointBlueRightPose;
    }
}
