package incognito.cog.component.drive;

import static incognito.cog.util.Generic.clamp;
import static incognito.cog.util.Generic.midpoint;
import static incognito.cog.util.Generic.withinThreshold;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.Arrays;

import incognito.cog.trajectory.TrajectorySequence;
import incognito.cog.trajectory.TrajectorySequenceBuilder;

public class TileCalculation {
    Drivetrain drivetrain;
    Pose2d lastPose;
    Tile targetTile;
    TrajectorySequenceBuilder sequenceBuilder = null;
    Trajectory lastTrajectory = null;
    Trajectory finalizingTrajectory = null;
    double MIN_X = -72; // in
    double MIN_Y = -72; // in
    double MAX_X = 72; // in
    double MAX_Y = 72; // in
    double CENTER_THRESHOLD = 1; // in

    /**
     * Create TileCalculations object to regulate movement between tiles.
     *
     * @param drivetrain The robot's drivetrain to create TrajectorySequences.
     */
    public TileCalculation(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        lastPose = drivetrain.getPoseEstimate();
        targetTile = getIDByVector();
    }

    enum FieldElement {
        A1, A12, A2, A23, A3, A34, A4, A45, A5, A56, A6,
        AB1, AB12, AB2, AB23, AB3, AB34, AB4, AB45, AB5, AB56, AB6,
        B1, B12, B2, B23, B3, B34, B4, B45, B5, B56, B6,
        BC1, BC12, BC2, BC23, BC3, BC34, BC4, BC45, BC5, BC56, BC6,
        C1, C12, C2, C23, C3, C34, C4, C45, C5, C56, C6,
        CD1, CD12, CD2, CD23, CD3, CD34, CD4, CD45, CD5, CD56, CD6,
        D1, D12, D2, D23, D3, D34, D4, D45, D5, D56, D6,
        DE1, DE12, DE2, DE23, DE3, DE34, DE4, DE45, DE5, DE56, DE6,
        E1, E12, E2, E23, E3, E34, E4, E45, E5, E56, E6,
        EF1, EF12, EF2, EF23, EF3, EF34, EF4, EF45, EF5, EF56, EF6,
        F1, F12, F2, F23, F3, F34, F4, F45, F5, F56, F6;

        static final int ROWS = 11;
        static final int COLS = 11;

        public static FieldElement getElement(int row, int col) {
            return FieldElement.values()[(row - 1) * COLS + (col - 1)];
        }

        public int getRow() {
            return this.ordinal() / COLS + 1;
        }

        public int getCol() {
            return this.ordinal() % COLS + 1;
        }

        public FieldElement nextRow() {
            int nextRow = clamp(this.getRow() + 1, 0, ROWS);
            return FieldElement.getElement(nextRow, this.getCol());
        }

        public FieldElement prevRow() {
            int prevRow = clamp(this.getRow() - 1, 0, ROWS);
            return FieldElement.getElement(prevRow, this.getCol());
        }

        public FieldElement nextCol() {
            int nextCol = clamp(this.getCol() + 1, 0, COLS);
            return FieldElement.getElement(this.getRow(), nextCol);
        }

        public FieldElement prevCol() {
            int prevCol = clamp(this.getCol() - 1, 0, COLS);
            return FieldElement.getElement(this.getRow(), prevCol);
        }

        /**
         * Find junction height by junction ID
         *
         * @param ID must be of style /[A-F]{2}[1-6]{2}/
         * @return Junction height (GROUND, LOW, MEDIUM, or HIGH) of the junction, or null
         * if the ID is not a junction.
         */
        public static Junction getJunctionHeightByID(FieldElement ID) {
            // GROUND
            if (Arrays.asList(
                    AB12, AB34, AB56,
                    CD12, CD34, CD56,
                    EF12, EF34, EF56
            ).contains(ID)) {
                return Junction.GROUND;
            }
            // LOW
            if (Arrays.asList(
                    AB23, AB45,
                    BC12, BC56,
                    DE12, DE56,
                    EF23, EF45
            ).contains(ID)) {
                return Junction.LOW;
            }

            // MEDIUM
            if (Arrays.asList(
                    BC23, BC45,
                    DE23, DE45
            ).contains(ID)) {
                return Junction.MEDIUM;
            }
            // HIGH
            if (Arrays.asList(
                    BC34,
                    CD23, CD45,
                    DE34
            ).contains(ID)) {
                return Junction.HIGH;
            }
            return null;
        }
    }

    public enum Move {
        UP,
        DOWN,
        LEFT,
        RIGHT,
    }

    public enum Junction {
        TOP_LEFT,
        TOP_RIGHT,
        BOTTOM_LEFT,
        BOTTOM_RIGHT,
        GROUND,
        LOW,
        MEDIUM,
        HIGH;

        public Junction getHeightByDirection(Tile tile) {
            return getHeightByDirection(this, tile);
        }

        public static Junction getHeightByDirection(Junction junction, Tile tile) {
            if (junction.ordinal() > 3) {
                // junction is already a height, not a direction
                return junction;
            }
            FieldElement fieldTile = FieldElement.valueOf(tile.name());
            FieldElement fieldJunction = null;
            if (junction == TOP_LEFT) {
                fieldJunction = fieldTile.prevRow().prevCol();
            } else if (junction == TOP_RIGHT) {
                fieldJunction = fieldTile.prevRow().nextCol();
            } else if (junction == BOTTOM_LEFT) {
                fieldJunction = fieldTile.nextRow().prevCol();
            } else if (junction == BOTTOM_RIGHT) {
                fieldJunction = fieldTile.nextRow().nextCol();
            }
            return FieldElement.getJunctionHeightByID(fieldJunction);
        }

        public static Junction getDirectionByHeight(Junction junction, Tile tile) {
            if (junction.ordinal() <= 3) {
                // Junction is already a direction, not a height
                return junction;
            }
            return null;
        }
    }

    enum Tile {
        A1, A2, A3, A4, A5, A6,
        B1, B2, B3, B4, B5, B6,
        C1, C2, C3, C4, C5, C6,
        D1, D2, D3, D4, D5, D6,
        E1, E2, E3, E4, E5, E6,
        F1, F2, F3, F4, F5, F6;

        static final int ROWS = 6;
        static final int COLS = 6;

        /**
         * Get a tile by row, column
         *
         * @param row Tile row (1-6)
         * @param col Tile column (1-6)
         * @return Tile associated with row, column
         */
        public static Tile getTile(int row, int col) {
            return Tile.values()[(row - 1) * COLS + (col - 1)];
        }

        public int[] getRowCol() {
            return new int[]{this.getRow(), this.getCol()};
        }

        public int getRow() {
            return this.ordinal() / COLS + 1;
        }

        public int getCol() {
            return this.ordinal() % COLS + 1;
        }

        public Tile nextRow() {
            int nextRow = clamp(this.getRow() + 1, 0, ROWS);
            return Tile.getTile(nextRow, this.getCol());
        }

        public Tile prevRow() {
            int prevRow = clamp(this.getRow() - 1, 0, ROWS);
            return Tile.getTile(prevRow, this.getCol());
        }

        public Tile nextCol() {
            int nextCol = clamp(this.getCol() + 1, 0, COLS);
            return Tile.getTile(this.getRow(), nextCol);
        }

        public Tile prevCol() {
            int prevCol = clamp(this.getCol() - 1, 0, COLS);
            return Tile.getTile(this.getRow(), prevCol);
        }
    }

    /**
     * Find what field tile a Vector2d is in by an ID with format /[A-F][1-6]/
     * col increases as x increases.
     * row increases as y decreases.
     *
     * @param pos The Vector2d (x, y) position.
     * @return Tile The Tile ID that the Vector2d is in.
     */
    public Tile getIDByVector(Vector2d pos) {
        // Clamp x and y within min/max bounds
        double x = clamp(pos.getX(), MIN_X, MAX_X);
        double y = -clamp(pos.getY(), MIN_Y, MAX_Y);
        // Get row/col based on x and y
        int row = ((int) y + 72 - 12) / 24 + 1;
        int col = ((int) x + 72 - 12) / 24 + 1;
        return Tile.getTile(row, col);
    }

    /**
     * Find what field tile the robot is currently in by an ID with format /[A-F][1-6]/
     *
     * @return Tile The Tile ID that the robot is currently in.
     */
    public Tile getIDByVector() {
        return getIDByVector(lastPose);
    }

    /**
     * Find what field tile the given Pose2d x and y is in by an ID with format /[A-F][1-6]/
     *
     * @param pose The Pose2d to convert to a Vector2d.
     * @return Tile The Tile ID that the Pose2d is in.
     */
    public Tile getIDByVector(Pose2d pose) {
        return getIDByVector(new Vector2d(pose.getX(), pose.getY()));
    }

    /**
     * Find the Roadrunner vector position of the center of a tile by ID.
     *
     * @param ID The tile ID to use.
     * @return Vector2d The (x, y) position of the center of the tile.
     */
    public Vector2d getVectorByID(Tile ID) {
        int row = ID.getRow();
        int col = ID.getCol();
        int y = -((row - 1) * 24 + 12 - 72);
        int x = (col - 1) * 24 + 12 - 72;
        return new Vector2d(x, y);
    }

    /**
     * Find the Roadrunner vector position of the center the current robot's tile.
     *
     * @return Vector2d The (x, y) position of the center of the tile.
     */
    public Vector2d getVectorByID() {
        return getVectorByID(getIDByVector());
    }

    /**
     * Determine if the robot is at (or near) the center of its current tile
     * with threshold CENTER_THRESHOLD.
     *
     * @return boolean Returns true if the robot is centered.
     */
    public boolean isCentered() {
        return isCentered(CENTER_THRESHOLD, getIDByVector());
    }

    /**
     * Determine if the robot is at (or near) the center of its current tile.
     *
     * @param threshold Number of inches as threshold to be considered centered.
     * @return boolean Returns true if the robot is centered.
     */
    public boolean isCentered(double threshold) {
        return isCentered(threshold, getIDByVector());
    }

    /**
     * Determine if the robot is at (or near) the center of a tile.
     *
     * @param threshold Number of inches as threshold to be considered centered.
     * @param ID        The Tile ID to check for centering.
     * @return boolean Returns true if the robot is centered.
     */
    public boolean isCentered(double threshold, Tile ID) {
        Vector2d currentPos = new Vector2d(lastPose.getX(), lastPose.getY());
        Vector2d targetPos = getVectorByID(ID);
        return withinThreshold(currentPos.getX(), targetPos.getX(), threshold) &&
                withinThreshold(currentPos.getY(), targetPos.getY(), threshold);
    }

    /**
     * Add a trajectory to the queue.
     *
     * @param traj Trajectory to add.
     */
    public void addTrajectory(Trajectory traj) {
        if (sequenceBuilder == null)
            sequenceBuilder = drivetrain.trajectorySequenceBuilder(lastPose);
        sequenceBuilder.addTrajectory(traj);
        lastTrajectory = traj;
    }

    /**
     * Queue a trajectory movement in a direction, ending in the center of the next tile.
     * Continuous.
     *
     * @param direction Move direction to move in (UP, DOWN, LEFT, or RIGHT)
     */
    public void queueMove(Move direction) {
        Tile nextTile;
        Trajectory newTrajectorySegment1;
        Trajectory newTrajectorySegment2;
        double startHeading;
        double endHeading;
        Pose2d startPose;
        switch (direction) {
            case UP:
                nextTile = targetTile.prevRow();
                startHeading = Math.toRadians(90);
                endHeading = Math.toRadians(90);
                break;
            case DOWN:
                nextTile = targetTile.nextRow();
                startHeading = Math.toRadians(270);
                endHeading = Math.toRadians(270);
                break;
            case LEFT:
                nextTile = targetTile.prevCol();
                startHeading = Math.toRadians(180);
                endHeading = Math.toRadians(180);
                break;
            case RIGHT:
                nextTile = targetTile.nextCol();
                startHeading = Math.toRadians(0);
                endHeading = Math.toRadians(0);
                break;
            default:
                return;
        }
        if (nextTile == targetTile) {
            return;
        }
        // Remove the last half-square trajectory
        if (lastTrajectory == null) {
            startPose = drivetrain.getPoseEstimate();
        } else {
            startHeading = lastTrajectory.end().getHeading();
            startPose = lastTrajectory.end();
        }


        // Add new full-square trajectory
        newTrajectorySegment1 = drivetrain.trajectoryBuilder(startPose, startHeading)
                .splineToConstantHeading(
                        midpoint(getVectorByID(nextTile), getVectorByID(targetTile)),
                        endHeading)
                .build();
        newTrajectorySegment2 = drivetrain.trajectoryBuilder(newTrajectorySegment1.end(), endHeading)
                .splineToConstantHeading(
                        getVectorByID(nextTile),
                        endHeading)
                .build();
        addTrajectory(newTrajectorySegment1);
        finalizingTrajectory = newTrajectorySegment2;
        targetTile = nextTile;
    }

    public void finalizeTrajectory() {
        addTrajectory(finalizingTrajectory);
    }

    public void reset() {
        sequenceBuilder = null;
        lastTrajectory = null;
        finalizingTrajectory = null;
    }

    public TrajectorySequence build() {
        if (sequenceBuilder == null) return null;
        return sequenceBuilder.build();
    }

    public void queueMoveToJunction(Junction junction) {
        if (junction.ordinal() > 3) {
            // Ordinal is GROUND, LOW, MEDIUM, or HIGH
            // Find junction direction of this height?
            return;
        }
        Junction height = junction.getHeightByDirection(targetTile);
        double[][] heightDistance = { // in
                {5, 5},
                {5, 5},
                {5, 5},
                {5, 5},
        };

        double[] endHeadings = { // Radians
                // Top Left
                Math.toRadians(135),
                // Top Right
                Math.toRadians(45),
                // Bottom Left
                Math.toRadians(225),
                // Bottom Right
                Math.toRadians(315)
        };
        double[][] factors = { // X, Y
                // Top Left
                {-1, 1},
                // Top Right
                {1, 1},
                // Bottom Left
                {-1, -1},
                // Bottom Right
                {1, -1}
        };

        Pose2d startPose;
        if (lastTrajectory != null) {
            startPose = lastTrajectory.end();
        } else {
            startPose = drivetrain.getPoseEstimate();
        }
        double endX = startPose.getX()
                + factors[junction.ordinal()][0]
                * heightDistance[height.ordinal() - 4][0];
        double endY = startPose.getY()
                + factors[junction.ordinal()][1]
                * heightDistance[height.ordinal() - 4][1];
        Pose2d endPose = new Pose2d(endX, endY, endHeadings[junction.ordinal()]);
        Trajectory moveToJunction = drivetrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(endPose)
                .build();
        finalizeTrajectory();
        addTrajectory(moveToJunction);
    }

    /**
     * Queue a trajectory to center the robot on the tile. Discontinuous.
     */
    public void queueCenter() {
        queueCenter(CENTER_THRESHOLD);
    }

    /**
     * Queue a trajectory to center the robot on the tile. Discontinuous.
     *
     * @param threshold The threshold (in inches) to be considered centered.
     */
    public void queueCenter(double threshold) {
        if (lastTrajectory != null) {
            return;
        }
        // No need to center the robot if already centered
        if (isCentered(threshold))
            return;

        Trajectory centerTrajectory = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                .lineToConstantHeading(getVectorByID())
                .build();
        addTrajectory(centerTrajectory);
    }

    /**
     * Clear the trajectory queue.
     */
    public void queueClear() {
        // TODO: Consider implementing cancellation of trajectories and cancel current movement here
        sequenceBuilder = null;
    }

    public boolean queueHasTrajectory() {
        return lastTrajectory != null;
    }

    public void update() {
        lastPose = drivetrain.getPoseEstimate();
        targetTile = getIDByVector();
    }
}