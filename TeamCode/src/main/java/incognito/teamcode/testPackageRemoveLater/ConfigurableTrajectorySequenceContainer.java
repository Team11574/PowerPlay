package incognito.teamcode.testPackageRemoveLater;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import incognito.cog.hardware.component.drive.Drivetrain;
import incognito.teamcode.config.DriveConstants;

public class ConfigurableTrajectorySequenceContainer {
    List<ConfigurableTrajectorySequenceBuilder> builds = new ArrayList<>();
    HashMap<String, Integer> buildNameMap = new HashMap<>();
    HashMap<Integer, String> buildIndexMap = new HashMap<>();
    boolean isFinished = false;

    public ConfigurableTrajectorySequenceContainer() {
        /*StackTraceElement[] trace = Thread.currentThread().getStackTrace();
        // Get name of the parent class that is calling this function (the class that is creating the container)
        parentName = trace[trace.length-1]
                .getClassName()
                .substring(
                        trace[trace.length-1]
                                .getClassName()
                                .lastIndexOf('.')+1
                );
                Not actually useful because FtcDashboard already parses the parent name for us
                */
    }

    public ConfigurableTrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        if (isFinished) return null;
        if (!buildIndexMap.containsKey(size())) {
            buildIndexMap.put(size(), "traj_" + size());
            buildNameMap.put("traj_" + size(), size());
        }
        ConfigurableTrajectorySequenceBuilder CTSB = new ConfigurableTrajectorySequenceBuilder(
                startPose,
                Drivetrain.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                Drivetrain.getAccelerationConstraint(DriveConstants.MAX_ACCEL),
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL
        );
        builds.add(CTSB);
        return CTSB;
    }

    public ConfigurableTrajectorySequenceBuilder trajectorySequenceBuilder(String name, Pose2d startPose) {
        if (isFinished) return null;
        buildNameMap.put(name, size());
        buildIndexMap.put(size(), name);
        return this.trajectorySequenceBuilder(startPose);
    }

    public ConfigurableTrajectorySequenceBuilder get(int index) {
        return builds.get(index);
    }

    public ConfigurableTrajectorySequenceBuilder get(String name) {
        if (buildNameMap.containsKey(name) && buildNameMap.get(name) != null && buildNameMap.get(name) < size()) {
            return builds.get(buildNameMap.get(name));
        }
        return null;
    }

    public int size() {
        if (builds.size() != buildIndexMap.size()
        || builds.size() != buildNameMap.size()
        || buildIndexMap.size() != buildNameMap.size()) {
            throw new RuntimeException("ConfigurableTrajectorySequenceContainer has inconsistent size"
            + "\nbuilds.size() = " + builds.size()
            + "\nbuildIndexMap.size() = " + buildIndexMap.size()
            + "\nbuildNameMap.size() = " + buildNameMap.size());
        }
        return builds.size();
    }

    public void end() {
        isFinished = true;
    }

    public boolean done() {
        return isFinished;
    }

    public String getNameByIndex(int index) {
        return buildIndexMap.get(index);
    }

    public CustomVariable getConfigurableTrajectorySequenceContainerVariable() {
        CustomVariable root = new CustomVariable();
        for (int i = 0; i < size(); i++) {
            root.putVariable(getNameByIndex(i), get(i).getConfigurableVariables());
        }
        return root;
    }

    @NonNull
    @Override
    public String toString() {
        return builds.toString();
    }
}
