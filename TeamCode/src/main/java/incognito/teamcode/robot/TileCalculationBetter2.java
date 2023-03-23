package incognito.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import incognito.cog.hardware.component.drive.Drivetrain;
import incognito.cog.trajectory.TrajectorySequence;
import incognito.cog.trajectory.TrajectorySequenceBuilder;
import incognito.cog.util.TelemetryBigError;


public class TileCalculationBetter2 {

    public enum TileDirection {
        UP, DOWN, LEFT, RIGHT,
        UP_LEFT, UP_RIGHT, DOWN_LEFT, DOWN_RIGHT,
        LEFT_UP, RIGHT_UP, LEFT_DOWN, RIGHT_DOWN;

        final List<String> STRINGS = Arrays.asList(
                "UP", "DOWN", "LEFT", "RIGHT",
                "UP_LEFT", "UP_RIGHT", "DOWN_LEFT", "DOWN_RIGHT",
                "LEFT_UP", "RIGHT_UP", "LEFT_DOWN", "RIGHT_DOWN"
        );
        final List<List<String>> STRINGS_SPLIT = Arrays.asList(
                Arrays.asList("UP"),
                Arrays.asList("DOWN"),
                Arrays.asList("LEFT"),
                Arrays.asList("RIGHT"),
                Arrays.asList("UP", "LEFT"),
                Arrays.asList("UP", "RIGHT"),
                Arrays.asList("DOWN", "LEFT"),
                Arrays.asList("DOWN", "RIGHT"),
                Arrays.asList("LEFT", "UP"),
                Arrays.asList("RIGHT", "UP"),
                Arrays.asList("LEFT", "DOWN"),
                Arrays.asList("RIGHT", "DOWN")
        );

        public TileDirection inverse() {
            switch (this) {
                case UP: return DOWN;
                case DOWN: return UP;
                case LEFT: return RIGHT;
                case RIGHT: return LEFT;
                case UP_LEFT: return DOWN_RIGHT;
                case UP_RIGHT: return DOWN_LEFT;
                case DOWN_LEFT: return UP_RIGHT;
                case DOWN_RIGHT: return UP_LEFT;
                default: return null;
            }
        }

        public double getHeading() {
            switch (this) {
                case RIGHT: return Math.toRadians(0);
                case UP: return Math.toRadians(90);
                case LEFT: return Math.toRadians(180);
                case DOWN: return Math.toRadians(270);
                // figure out if other headings are needed
                default: return -1;
            }
        }

        public boolean isTowards(TileDirection direction) {
            return STRINGS_SPLIT.get(this.ordinal()).contains(direction.name());
        }


        public double getX() {
            if (this.isTowards(RIGHT)) return TILE_SIZE/2;
            if (this.isTowards(LEFT)) return -TILE_SIZE/2;
            return 0;
        }

        public double getY() {
            if (this.isTowards(UP)) return TILE_SIZE/2;
            if (this.isTowards(DOWN)) return -TILE_SIZE/2;
            return 0;
        }

        public Vector2d getVector() {
            return new Vector2d(getX(), getY());
        }

        public TileDirection and(TileDirection direction) {
            String combination = this.name() + "_" + direction.name();
            if (STRINGS.contains(combination)) {
                return TileDirection.valueOf(combination);
            }
            // This is bad
            return direction;
        }
    }

    public enum BuildState {
        STARTED, ACTIVE, ADJUSTED, CANCELLED, FINISHED, INACTIVE, ERROR
    }

    Drivetrain drivetrain;
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
    public TileCalculationBetter2(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /**
     * Build index is the index of the final value in the trajectory list.
     * @return int
     */
    public int getBuildIndex() {
        return directions.size() - 1;
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
     * Add a new trajectory to the list.
     */
    public void push(TileDirection direction) {
        directions.add(direction);
    }

    /**
     * Remove the last trajectory and direction from the list.
     */
    public TileDirection pop() {
        if (directions.size() == 0) {
            return null;
        }
        return directions.remove(directions.size() - 1);
    }

    public Pose2d getSequenceStartPose() {
        if (sequenceStartPose == null) {
            // If we are just starting a new trajectory, we need to create a new pose
            sequenceStartPose = drivetrain.getPoseEstimate();
        }
        return sequenceStartPose;
    }

    /**
     * Move the robot 24 inches in the given direction.
     *
     * @param direction The direction to move the robot.
     */
    public void move(TileDirection direction) {
        if (direction.inverse() == getLastDirection()) {
            // If the robot is moving in the opposite direction of the last direction,
            // then we want to remove the last movement.
            pop();
            pop();
        } else if (direction == getLastDirection() || getLastDirection() == null) {
            // If the robot is moving in the same direction as the last direction,
            // then we want to add a movement.
            push(direction);
            push(direction);
        } else {
            pop();
            push(getLastDirection().and(direction));
            push(direction);
        }
    }

    public TrajectorySequence build() {
        if (directions.size() == 0) {
            return null;
        }
        TrajectorySequenceBuilder sequenceBuilder = drivetrain.trajectorySequenceBuilder(sequenceStartPose);
        Vector2d lastPos = new Vector2d(sequenceStartPose.getX(), sequenceStartPose.getY());
        for (int i = 0; i < directions.size(); i++) {
            TileDirection direction = directions.get(i);
            if (i == 0) sequenceBuilder = sequenceBuilder.setTangent(direction.getHeading());
            sequenceBuilder = sequenceBuilder.splineToConstantHeading(
                    lastPos.plus(direction.getVector()),
                    direction.getHeading()
            );
            lastPos = lastPos.plus(direction.getVector());
        }
        lastBuiltIndex = getBuildIndex();
        return sequenceBuilder.build();
    }
}