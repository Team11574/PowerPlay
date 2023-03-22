package incognito.teamcode.robot;

import static incognito.cog.util.Generic.clamp;
import static incognito.cog.util.Generic.midpoint;
import static incognito.cog.util.Generic.withinThreshold;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.Arrays;

import incognito.cog.trajectory.TrajectorySequenceBuilder;
import incognito.cog.util.TelemetryBigError;
import incognito.cog.hardware.component.drive.Drivetrain;
import incognito.cog.trajectory.TrajectorySequence;


public class TileCalculationBetter {

    public enum TileDirection {
        UP, DOWN, LEFT, RIGHT;

        public TileDirection inverse() {
            switch (this) {
                case UP: return DOWN;
                case DOWN: return UP;
                case LEFT: return RIGHT;
                case RIGHT: return LEFT;
                default: return null;
            }
        }

        public double getHeading() {
            switch (this) {
                case RIGHT: return Math.toRadians(0);
                case UP: return Math.toRadians(90);
                case LEFT: return Math.toRadians(180);
                case DOWN: return Math.toRadians(270);
                default: return 0;
            }
        }

        public double getX() {
            switch (this) {
                case RIGHT: return TILE_SIZE/2;
                case UP: return 0;
                case LEFT: return -TILE_SIZE/2;
                case DOWN: return 0;
                default: return 0;
            }
        }

        public double getY() {
            switch (this) {
                case RIGHT: return 0;
                case UP: return TILE_SIZE/2;
                case LEFT: return 0;
                case DOWN: return -TILE_SIZE/2;
                default: return 0;
            }
        }
    }

    public enum BuildState {
        STARTED, ACTIVE, ADJUSTED, CANCELLED, FINISHED, INACTIVE, ERROR
    }

    Drivetrain drivetrain;
    ArrayList<Trajectory> trajectories = new ArrayList<>();
    ArrayList<TileDirection> directions = new ArrayList<>();
    int lastBuiltIndex = -1;
    Pose2d sequenceStartPose;
    static final double TILE_SIZE = 24;
    static final double MIN_X = -72; // in
    static final double MIN_Y = -72; // in
    static final double MAX_X = 72; // in
    static final double MAX_Y = 72; // in

    /**
     * Create TileCalculation object to regulate movement between tiles.
     *
     * @param drivetrain The robot's drivetrain used to create TrajectorySequences.
     */
    public TileCalculationBetter(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /**
     * Build index is the index of the final value in the trajectory list.
     * @return int
     */
    public int getBuildIndex() {
        return trajectories.size() - 1;
    }

    /**
     * Last built index is the index of the last trajectory that has already been built.
     * @return int
     */
    public int getLastBuiltIndex() {
        return lastBuiltIndex;
    }

    /**
     * Get the current build state of the TrajectorySequence.
     *
     * @param activeSegmentIndex The current segment index of the active TrajectorySequence.
     * @return The current build state of the TrajectorySequence.
     */
    public BuildState getBuildState(Integer activeSegmentIndex) {
        if (activeSegmentIndex == null) {
            activeSegmentIndex = -1;
        }
        if (activeSegmentIndex == -1
                && getBuildIndex() >= 0
                && getLastBuiltIndex() == -1) {
            // Build has just started since the drive is not running a trajectory
            // but we have trajectories ready to be built but have not built any yet.
            return BuildState.STARTED;
        } else if (activeSegmentIndex >= 0
                && getBuildIndex() == getLastBuiltIndex()) {
            // Build is active and the drive is running a trajectory that has already been built.
            return BuildState.ACTIVE;
        } else if (activeSegmentIndex >= 0
                && getBuildIndex() != getLastBuiltIndex()
                && activeSegmentIndex < getBuildIndex()) {
            // Build is active but we have added a new trajectory to the list,
            // and the drive is still running a trajectory that was built before.
            // TODO: It's possible that if the activeSegmentIndex = getLastBuiltIndex() then
            //  then adding a new trajectory will create a pause in between, so this may need
            //  to be tested or changed.
            return BuildState.ADJUSTED;
        } else if (activeSegmentIndex >= 0
                && getBuildIndex() != getLastBuiltIndex()
                && activeSegmentIndex >= getBuildIndex()) {
            // Build is actively following a trajectory that we want to change,
            // which is bad.
            return BuildState.CANCELLED;
        } else if (activeSegmentIndex == -1
                && getBuildIndex() == getLastBuiltIndex()
                && getBuildIndex() != -1) {
            // Build has just finished since the drive is not running a trajectory
            // but previously we had built all of the trajectories available.
            reset();
            return BuildState.FINISHED;
        } else if (activeSegmentIndex == -1
                && getBuildIndex() == -1
                && getLastBuiltIndex() == -1) {
            // Build is inactive since the drive is not running a trajectory
            // and we have not built any trajectories yet.
            return BuildState.INACTIVE;
        }
        // If none of the above cases are true, then we haven't accounted for something.
        TelemetryBigError.raise(1);
        // reset();
        return BuildState.ERROR;
    }

    /**
     * Reset the TileCalculation object.
     */
    public void reset() {
        trajectories.clear();
        directions.clear();
        lastBuiltIndex = -1;
        sequenceStartPose = null;
    }

    /**
     * Get the last direction that the robot moved in.
     * @return The last direction that the robot moved in, or null if the robot hasn't moved.
     */
    public TileDirection getLastDirection() {
        if (directions.size() == 0) {
            return null;
        }
        return directions.get(directions.size() - 1);
    }

    /**
     * Get the last trajectory that was added to the list.
     * @return The last trajectory that was added to the list, or null if the list is empty.
     */
    public Trajectory getLastTrajectory() {
        if (trajectories.size() == 0) {
            return null;
        }
        return trajectories.get(trajectories.size() - 1);
    }

    /**
     * Add a new trajectory to the list.
     */
    public void push(Trajectory trajectory, TileDirection direction) {
        trajectories.add(trajectory);
        directions.add(direction);
    }

    /**
     * Remove the last trajectory and direction from the list.
     */
    public void pop() {
        if (trajectories.size() == 0) {
            return;
        }
        trajectories.remove(trajectories.size() - 1);
        directions.remove(directions.size() - 1);
    }

    public Pose2d getSequenceStartPose() {
        if (sequenceStartPose == null) {
            // If we are just starting a new trajectory, we need to create a new pose
            sequenceStartPose = drivetrain.getPoseEstimate();
        }
        return sequenceStartPose;
    }

    /**
     * Move the robot to the next tile (a move of 24 inches).
     *
     * @param direction The direction to move the robot.
     */
    public void move(TileDirection direction) {
        move(direction, 2);
    }

    /**
     * Move the robot 12 inches in the given direction.
     *
     * @param direction The direction to move the robot.
     * @param times The number of times to move 12 inches in the given direction.
     */
    public void move(TileDirection direction, int times) {
        if (direction.inverse() == getLastDirection()) {
            // If the robot is moving in the opposite direction of the last direction,
            // then we want to remove the last movement.
            pop();
        } else {
            // If the robot is not moving in the same direction as the last direction,
            // then we want to add a movement.
            Trajectory lastTrajectory = getLastTrajectory();
            Pose2d startPose;
            if (lastTrajectory == null) {
                startPose = getSequenceStartPose();
            } else {
                startPose = lastTrajectory.end();
            }
            double startX = startPose.getX();
            double startY = startPose.getY();
            double endX = startX + direction.getX();
            double endY = startY + direction.getY();
            // Account for out of bounds
            if (endX <= MIN_X || endX >= MAX_X) {
                endX = startX;
            }
            if (endY <= MIN_Y || endY >= MAX_Y) {
                endY = startY;
            }

            push(drivetrain.trajectoryBuilder(startPose, direction.getHeading())
                            .splineToConstantHeading(
                                    new Vector2d(endX, endY), direction.getHeading()
                            ).build(),
                    direction);
        }
        if (times > 1) {
            move(direction, times - 1);
        }
    }

    public TrajectorySequence build() {
        if (trajectories.size() == 0) {
            return null;
        }
        Trajectory startTrajectory = trajectories.get(0);
        TrajectorySequenceBuilder sequenceBuilder = drivetrain.trajectorySequenceBuilder(sequenceStartPose)
                .setTangent(startTrajectory.start().getHeading())
                .addTrajectory(startTrajectory);
        for (Trajectory followingTrajectory : trajectories) {
            sequenceBuilder = sequenceBuilder.addTrajectory(followingTrajectory);
        }
        return sequenceBuilder.build();
    }
}