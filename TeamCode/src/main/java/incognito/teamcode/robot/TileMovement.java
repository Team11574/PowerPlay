package incognito.teamcode.robot;

import static incognito.teamcode.robot.TileMovement.BuildState.ACTIVE;
import static incognito.teamcode.robot.TileMovement.BuildState.INACTIVE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import incognito.cog.hardware.component.drive.Drivetrain;
import incognito.cog.trajectory.TrajectorySequence;
import incognito.cog.trajectory.TrajectorySequenceBuilder;
import incognito.cog.util.Generic;
import incognito.cog.util.TelemetryBigError;


public class TileMovement {

    public enum MoveDirection {
        // Cardinal moves
        UP, DOWN, LEFT, RIGHT,
        // L shaped moves
        UP_LEFT, UP_RIGHT, DOWN_LEFT, DOWN_RIGHT,
        LEFT_UP, RIGHT_UP, LEFT_DOWN, RIGHT_DOWN,
        // Junctions
        J_UP_LEFT, J_UP_RIGHT, J_DOWN_LEFT, J_DOWN_RIGHT,
        CENTER;

        final List<String> STRINGS = Arrays.asList(
                "UP", "DOWN", "LEFT", "RIGHT",
                "UP_LEFT", "UP_RIGHT", "DOWN_LEFT", "DOWN_RIGHT",
                "LEFT_UP", "RIGHT_UP", "LEFT_DOWN", "RIGHT_DOWN",
                "J_UP_LEFT", "J_UP_RIGHT", "J_DOWN_LEFT", "J_DOWN_RIGHT", "CENTER"
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
                Arrays.asList("RIGHT", "DOWN"),

                Arrays.asList("J", "UP", "LEFT"),
                Arrays.asList("J", "UP", "RIGHT"),
                Arrays.asList("J", "DOWN", "LEFT"),
                Arrays.asList("J", "DOWN", "RIGHT"),
                Arrays.asList("CENTER")
        );

        public MoveDirection inverse() {
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
                case UP_RIGHT:
                case DOWN_RIGHT:
                case RIGHT: return Math.toRadians(0);
                case RIGHT_UP:
                case LEFT_UP:
                case UP: return Math.toRadians(90);
                case UP_LEFT:
                case DOWN_LEFT:
                case LEFT: return Math.toRadians(180);
                case LEFT_DOWN:
                case RIGHT_DOWN:
                case DOWN: return Math.toRadians(270);
                // Junctions
                case J_UP_RIGHT: return Math.toRadians(45);
                case J_UP_LEFT: return Math.toRadians(135);
                case J_DOWN_RIGHT: return Math.toRadians(225);
                case J_DOWN_LEFT: return Math.toRadians(315);
                // something is wrong if we get here
                default: return -1;

            }
        }

        public boolean isTowards(MoveDirection direction) {
            return STRINGS_SPLIT.get(this.ordinal()).contains(direction.name());
        }

        public boolean isJunctionMove() {
            return this.name().startsWith("J");
        }

        public boolean isCenter() { return this == CENTER; }


        public double getX() {
            if (this.isJunctionMove()) {
                return Math.signum(Math.cos(this.getHeading()));
            }
            if (this.isTowards(RIGHT)) return TILE_SIZE/2;
            if (this.isTowards(LEFT)) return -TILE_SIZE/2;
            return 0;
        }

        public double getY() {
            if (this.isJunctionMove()) {
                return Math.signum(Math.sin(this.getHeading()));
            }
            if (this.isTowards(UP)) return TILE_SIZE/2;
            if (this.isTowards(DOWN)) return -TILE_SIZE/2;
            return 0;
        }

        public Vector2d getVector() {
            return new Vector2d(getX(), getY());
        }

        public MoveDirection and(MoveDirection direction) {
            String combination = this.name() + "_" + direction.name();
            if (STRINGS.contains(combination)) {
                return MoveDirection.valueOf(combination);
            }
            // This is bad
            return direction;
        }
    }

    public enum Junction {
        GROUND,
        LOW,
        MEDIUM,
        HIGH,
        NONE;

        public static final Junction[][] junctionOrder = {
                {GROUND, LOW, GROUND, LOW, GROUND},
                {LOW, MEDIUM, HIGH, MEDIUM, LOW},
                {GROUND, HIGH, GROUND, HIGH, GROUND},
                {LOW, MEDIUM, HIGH, MEDIUM, LOW},
                {GROUND, LOW, GROUND, LOW, GROUND},
        };

        public static Junction getJunctionTowards(double x, double y, double heading) {
            return getJunctionTowards(new Pose2d(x, y, heading));
        }

        public static Junction getJunctionTowards(Vector2d vector, double heading) {
            return  getJunctionTowards(new Pose2d(vector, heading));
        }

        public static Junction getJunctionTowards(Pose2d pose) {
            // Assume pose position is centered on a tile
            // Shift in the direction of the corner and find the nearest junction
            return getJunctionAtPosition(
                    pose.getX() + TILE_SIZE/2 * Math.cos(pose.getHeading()) * Math.sqrt(2),
                    pose.getY() + TILE_SIZE/2 * Math.sin(pose.getHeading()) * Math.sqrt(2)
            );
        }

        public static Junction getJunctionAtPosition(Pose2d pose) {
            return getJunctionAtPosition(pose.getX(), pose.getY());
        }

        public static Junction getJunctionAtPosition(Vector2d vector) {
            return getJunctionAtPosition(vector.getX(), vector.getY());
        }
        public static Junction getJunctionAtPosition(double x, double y) {
            int xIndex = (Generic.roundToFactor(x + 72, 24)) / 24 - 1;
            int yIndex = (Generic.roundToFactor(y + 72, 24)) / 24 - 1;
            // If index is out of bounds, return NONE
            if (xIndex < 0 || xIndex > junctionOrder[0].length || yIndex < 0 || yIndex > junctionOrder.length) {
                return NONE;
            }
            return junctionOrder[yIndex][xIndex];
        }

        public double getHeadingOffset() {
            if (this == GROUND) return Math.toRadians(180);
            return 0;
        }

        public double getXOffset() {
            switch (this) {
                case GROUND: return -4;
                case HIGH: return 5;
                case MEDIUM: return 6;
                case LOW: return 7;
            }
            return 0;
        }
        public double getYOffset() {
            // I'm lazy okay and it's probably the same anyway
            return getXOffset();
        }
    }

    public enum BuildState {
        STARTED, ACTIVE, ADJUSTED, CANCELLED, FINISHED, INACTIVE, ERROR
    }

    Drivetrain drivetrain;
    ArrayList<MoveDirection> directions = new ArrayList<>();
    int lastBuiltIndex = -1;
    Pose2d sequenceStartPose;
    static final double TILE_SIZE = 24; // in
    static final double EDGE_THRESHOLD = 8; // half of the robot width (roughly), in
    static final double MIN_X = -(72 - EDGE_THRESHOLD) ; // in
    static final double MIN_Y = -(72 - EDGE_THRESHOLD); // in
    static final double MAX_X = 72 - EDGE_THRESHOLD; // in
    static final double MAX_Y = 72 - EDGE_THRESHOLD; // in

    /**
     * Create TileCalculation object to regulate movement between tiles.
     *
     * @param drivetrain The robot's drivetrain used to create TrajectorySequences.
     */
    public TileMovement(Drivetrain drivetrain) {
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


    /*
    switch (tileMovement.getBuildState(drivetrain.getCurrentSegmentIndex())) {
        case STARTED:
            drivetrain.followTrajectorySequenceAsync(tileMovement.build());
            break;
        case ADJUSTED:
            drivetrain.modifyTrajectorySequenceAsync(tileMovement.build());
            break;
        case ACTIVE:
            // dont go to manual mode
            if (buildStates.size() > 0 && buildStates.get(buildStates.size()-1) == ACTIVE) {
                buildStates.remove(buildStates.size()-1);
            }
            break;
        case INACTIVE:
            // stay in manual mode?
            if (buildStates.size() > 0 && buildStates.get(buildStates.size()-1) == INACTIVE) {
                buildStates.remove(buildStates.size()-1);
            }
            break;
        case CANCELLED:
            break;
        case FINISHED:
            // go to manual mode?
            break;
        case ERROR:
            multiTelemetry.addData("Error", "Error building trajectory");
            TelemetryBigError.raise(3);
            break;
    }
    */

    /**
     * Get the current build state of the TrajectorySequence.
     *
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

    public boolean hasMoves() {
        return directions.size() > 0;
    }

    /**
     * Get the last direction that the robot moved in.
     * @return The last direction that the robot moved in, or null if the robot hasn't moved.
     */
    public MoveDirection getLastDirection() {
        if (directions.size() == 0) {
            return null;
        }
        return directions.get(directions.size() - 1);
    }

    /**
     * Add a new trajectory to the list.
     */
    public void push(MoveDirection direction) {
        directions.add(direction);
    }

    /**
     * Remove the last trajectory and direction from the list.
     */
    public MoveDirection pop() {
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
    public TileMovement move(MoveDirection direction) {
        // Clear any accidental junction moves if a new directional move is queued
        if (getLastDirection() != null && getLastDirection().isJunctionMove()) pop();
        if (direction.isJunctionMove()) {
            moveToJunction(direction);
            return this;
        }
        if (direction.isCenter()) {
            moveCenter();
            return this;
        }
        if (directions.size() == 0) {
            // Center if first move
            moveCenter();
        }
        if (direction.inverse() == getLastDirection()) {
            // If the robot is moving in the opposite direction of the last direction,
            // then we want to remove the last movement.
            pop();
            pop();
        } else if (direction == getLastDirection()
                || getLastDirection() == null
                || getLastDirection() == MoveDirection.CENTER) {
            // If the robot is moving in the same direction as the last direction or
            // if we just started moving, then we want to add movement like normal.
            push(direction);
            push(direction);
        } else {
            pop();
            push(getLastDirection().and(direction));
            push(direction);
        }
        return this;
    }

    public TileMovement moveToJunction(MoveDirection direction) {
        // Clear previous junction move (only one allowed per trajectory)
        if (getLastDirection() != null && getLastDirection().isJunctionMove()) pop();
        if (!direction.isJunctionMove()) {
            return move(direction);
        }
        if (directions.size() == 0) {
            // Center if first move
            moveCenter();
        }
        push(direction);
        return this;
    }

    public TileMovement moveCenter() {
        // Only allow move to center if it is the first move in the sequence
        if (directions.size() != 0) return this;
        push(MoveDirection.CENTER);
        return this;
    }

    private TrajectorySequenceBuilder addMove(
            TrajectorySequenceBuilder sequenceBuilder,
            MoveDirection direction,
            Vector2d lastPos) {
        Vector2d addition = lastPos.plus(direction.getVector());
        if (addition.getX() > MAX_X
                || addition.getX() < MIN_X
                || addition.getY() > MAX_Y
                || addition.getY() < MIN_Y) {
            return sequenceBuilder;
        }
        return sequenceBuilder.splineToConstantHeading(
                lastPos.plus(direction.getVector()),
                direction.getHeading()
        );
    }

    private TrajectorySequenceBuilder addMoveToJunction(
            TrajectorySequenceBuilder sequenceBuilder,
            MoveDirection junctionDirection,
            Vector2d lastPos) {
        Junction junction = Junction.getJunctionTowards(lastPos, junctionDirection.getHeading());
        Vector2d offset = new Vector2d(
                junction.getXOffset() * junctionDirection.getX(),
                junction.getYOffset() * junctionDirection.getY()
        );
        double offsetHeading = (junction.getHeadingOffset() * junctionDirection.getHeading()) % (2*Math.PI);
        return sequenceBuilder.splineToSplineHeading(
                new Pose2d(
                        lastPos.plus(offset),
                        offsetHeading
                ), offsetHeading
        );
    }

    private TrajectorySequenceBuilder addMoveCenter(
            TrajectorySequenceBuilder sequenceBuilder,
            Vector2d lastPos,
            double endHeading) {
        return sequenceBuilder.splineToConstantHeading(
            new Vector2d(
                    Generic.roundToFactor(lastPos.getX() + 12, 24) - 12,
                    Generic.roundToFactor(lastPos.getY() + 12, 24) - 12
            ), endHeading
        );
    }

    public TrajectorySequence build() {
        if (directions.size() == 0) {
            return null;
        }
        TrajectorySequenceBuilder sequenceBuilder = drivetrain.trajectorySequenceBuilder(getSequenceStartPose());
        Vector2d lastPos = new Vector2d(getSequenceStartPose().getX(), getSequenceStartPose().getY());
        for (int i = 0; i < directions.size(); i++) {
            MoveDirection direction = directions.get(i);
            // Set tangent for first run
            if (i == 0) {
                sequenceBuilder = sequenceBuilder.setTangent(direction.getHeading());
                // If we explicitly only want to center and have no other actions, do as such
                if (direction.isCenter() && directions.size() == 1) {
                    sequenceBuilder = addMoveCenter(
                            sequenceBuilder,
                            lastPos,
                            getSequenceStartPose().getHeading()
                    );
                // Otherwise, it is an implicit centering that can be smooth
                } else if (!direction.isCenter()) {
                    // Add a center move to start
                    directions.add(0, MoveDirection.CENTER);
                    // Reset the counter
                    i--;
                    continue;
                // First direction is center and there are other directions after
                } else {
                    sequenceBuilder = addMoveCenter(
                            sequenceBuilder,
                            lastPos,
                            directions.get(i+1).getHeading()
                    );
                }
            }
            if (direction.isJunctionMove()) {
                sequenceBuilder = addMoveToJunction(sequenceBuilder, direction, lastPos);
            } else if (!direction.isCenter()){
                sequenceBuilder = addMove(sequenceBuilder, direction, lastPos);
            }
            lastPos = lastPos.plus(direction.getVector());
        }
        lastBuiltIndex = getBuildIndex();
        return sequenceBuilder.build();
    }
}