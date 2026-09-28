package frc.robot.data;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;
import frc.robot.Constants.FieldConstants;

public class FieldPoses {
    public static double getDistanceToHub(Pose2d pose) {
        Pose2d hub = getAllianceHubPose2d();
        return Math.hypot(pose.getX() - hub.getX(), pose.getY() - hub.getY());
    }

    public static Pose2d getAllianceHubPose2d() {
        return MatchState.getAlliance().get().equals(Alliance.RED) ? FieldConstants.kRedHubPose
                : FieldConstants.kBlueHubPose;
    }

    public static Pose2d getAllianceLeftPassingPoint() {
        return MatchState.getAlliance().get().equals(Alliance.RED)
                ? FieldConstants.kPassingPointRedLeftPose
                : FieldConstants.kPassingPointBlueLeftPose;
    }

    public static Pose2d getAllianceRightPassingPoint() {
        return MatchState.getAlliance().get().equals(Alliance.RED)
                ? FieldConstants.kPassingPointRedRightPose
                : FieldConstants.kPassingPointBlueRightPose;
    }
}
