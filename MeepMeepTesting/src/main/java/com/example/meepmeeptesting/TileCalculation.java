package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Generic.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.roadrunner.DriveShim;


import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.PriorityQueue;
import java.util.Queue;

public class TileCalculation {
    DriveShim drivetrain;
    Pose2d lastPose;
    Tile targetTile;
    //ArrayList<Trajectory> trajectoryQueue;
    ArrayList<returnThingy> trajectoryQueue;
    double MIN_X = -72; // in
    double MIN_Y = -72; // in
    double MAX_X = 72; // in
    double MAX_Y = 72; // in
    double CENTER_THRESHOLD = 1; // in

    /**
     * Create TileCalculations object to regulat movement between tiles.
     * @param drivetrain The robot's drivetrain to create TrajectorySequences.
     */
    public TileCalculation(DriveShim drivetrain) {
        this.drivetrain = drivetrain;
        lastPose = drivetrain.getPoseEstimate();
        targetTile = getIDByVector();
        trajectoryQueue = new ArrayList<>();
    }

    enum Field {
        A1, A12, A2, A23, A3, A34, A4, A45, A5, A56, A6,
        AB1,AB12,AB2,AB23,AB3,AB34,AB4,AB45,AB5,AB56,AB6,
        B1, B12, B2, B23, B3, B34, B4, B45, B5, B56, B6,
        BC1,BC12,BC2,BC23,BC3,BC34,BC4,BC45,BC5,BC56,BC6,
        C1, C12, C2, C23, C3, C34, C4, C45, C5, C56, C6,
        CD1,CD12,CD2,CD23,CD3,CD34,CD4,CD45,CD5,CD56,CD6,
        D1, D12, D2, D23, D3, D34, D4, D45, D5, D56, D6,
        DE1,DE12,DE2,DE23,DE3,DE34,DE4,DE45,DE5,DE56,DE6,
        E1, E12, E2, E23, E3, E34, E4, E45, E5, E56, E6,
        EF1,EF12,EF2,EF23,EF3,EF34,EF4,EF45,EF5,EF56,EF6,
        F1, F12, F2, F23, F3, F34, F4, F45, F5, F56, F6,
    }

    enum Move {
        UP,
        DOWN,
        LEFT,
        RIGHT,
    }

    enum Tile {
        A1, A2, A3, A4, A5, A6,
        B1, B2, B3, B4, B5, B6,
        C1, C2, C3, C4, C5, C6,
        D1, D2, D3, D4, D5, D6,
        E1, E2, E3, E4, E5, E6,
        F1, F2, F3, F4, F5, F6;

        /**
         * Get a tile by row, column
         * @param row Tile row (1-6)
         * @param col Tile column (1-6)
         * @return Tile associated with row, column
         */
        public static Tile getTile(int row, int col) {
            return Tile.values()[(row-1) * 6 + (col-1)];
        }

        public int[] getRowCol() {
            return new int[]{this.getRow(), this.getCol()};
        }

        public int getRow() {
            return this.ordinal() / 6 + 1;
        }

        public int getCol() {
            return this.ordinal() % 6 + 1;
        }

        public Tile nextRow() {
            int nextRow = clamp(this.getRow() + 1, 0, 6);
            return Tile.getTile(nextRow, this.getCol());
        }

        public Tile prevRow() {
            int prevRow = clamp(this.getRow() - 1, 0, 6);
            return Tile.getTile(prevRow, this.getCol());
        }

        public Tile nextCol() {
            int nextCol = clamp(this.getCol() + 1, 0, 6);
            return Tile.getTile(this.getRow(), nextCol);
        }

        public Tile prevCol() {
            int prevCol = clamp(this.getCol() - 1, 0, 6);
            return Tile.getTile(this.getRow(), prevCol);
        }
    }

    /**
     * Find what field tile a Vector2d is in by an ID with format /[A-F][1-6]/
     * col increases as x increases.
     * row increases as y decreases.
     * @param pos The Vector2d (x, y) position.
     * @return Tile The Tile ID that the Vector2d is in.
     */
    public Tile getIDByVector(Vector2d pos) {
        // Clamp x and y within min/max bounds
        double x =  clamp(pos.getX(), MIN_X, MAX_X);
        double y = -clamp(pos.getY(), MIN_Y, MAX_Y);
        // Get row/col based on x and y
        int row = ((int) y + 72 - 12) / 24 + 1;
        int col = ((int) x  + 72 - 12) / 24 + 1;
        return Tile.getTile(row, col);
    }

    /**
     * Find what field tile the robot is currently in by an ID with format /[A-F][1-6]/
     * @return Tile The Tile ID that the robot is currently in.
     */
    public Tile getIDByVector() {
        return getIDByVector(lastPose);
    }

    /**
     * Find what field tile the given Pose2d x and y is in by an ID with format /[A-F][1-6]/
     * @param pose The Pose2d to convert to a Vector2d.
     * @return Tile The Tile ID that the Pose2d is in.
     */
    public Tile getIDByVector(Pose2d pose) {
        return getIDByVector(new Vector2d(pose.getX(), pose.getY()));
    }

    /**
     * Find the Roadrunner vector position of the center of a tile by ID.
     * @param ID The tile ID to use.
     * @return Vector2d The (x, y) position of the center of the tile.
     */
    public Vector2d getVectorByID(Tile ID) {
        int row = ID.getRow();
        int col = ID.getCol();
        int y = -((row-1) * 24 + 12 - 72);
        int x = (col-1) * 24 + 12 - 72;
        return new Vector2d(x, y);
    }

    /**
     * Determine if the robot is at (or near) the center of its current tile
     * with threshold CENTER_THRESHOLD.
     * @return boolean Returns true if the robot is centered.
     */
    public boolean isCentered() {
        return isCentered(CENTER_THRESHOLD, getIDByVector());
    }

    /**
     * Determine if the robot is at (or near) the center of its current tile.
     * @param threshold Number of inches as threshold to be considered centered.
     * @return boolean Returns true if the robot is centered.
     */
    public boolean isCentered(double threshold) {
        return isCentered(threshold, getIDByVector());
    }

    /**
     * Determine if the robot is at (or near) the center of a tile.
     * @param threshold Number of inches as threshold to be considered centered.
     * @param ID The Tile ID to check for centering.
     * @return boolean Returns true if the robot is centered.
     */
    public boolean isCentered(double threshold, Tile ID) {
        Vector2d currentPos = new Vector2d(lastPose.getX(), lastPose.getY());
        Vector2d targetPos = getVectorByID(ID);
        return withinThreshold(currentPos.getX(), targetPos.getX(), threshold) &&
                withinThreshold(currentPos.getY(), targetPos.getY(), threshold);
    }

    // queueMoveLeft();
    // queueMove(TileCalculation.Move.LEFT);

    public void queueAdd(Trajectory traj) {
        //trajectoryQueue.add(traj);
    }

    public returnThingy queueMove(Move direction) {
        Tile nextTile;
        Trajectory newTrajectorySegment1;
        Trajectory newTrajectorySegment2;
        Trajectory lastTrajectory;
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
                return null;
        }
        if (nextTile == targetTile) {
            return null;
        }
        // Remove the last half-square trajectory
        if (trajectoryQueue.size() < 2) {
            startPose = drivetrain.getPoseEstimate();
        } else {
            /*
            lastTrajectory = trajectoryQueue.remove(trajectoryQueue.size() - 1);
            startHeading = lastTrajectory.end().getHeading();
            startPose = trajectoryQueue.get(trajectoryQueue.size() - 1).end();
             */
            startHeading = trajectoryQueue.remove(trajectoryQueue.size() - 1).startHeading;
            startPose = trajectoryQueue.get(trajectoryQueue.size() - 1).startPose;
        }
        returnThingy toReturn = new returnThingy(nextTile, startHeading, endHeading, startPose);
        trajectoryQueue.add(toReturn);
        return toReturn;


        // Add new full-square trajectory
        /*
        newTrajectorySegment1 = drivetrain.trajectoryBuilder(startPose, startHeading)
                .splineToConstantHeading(
                        midpoint(getVectorByID(nextTile), getVectorByID(targetTile)),
                        endHeading)
                .build();
        newTrajectorySegment2 = drivetrain.trajectoryBuilder(newTrajectorySegment1.end())
                .splineToConstantHeading(
                        getVectorByID(nextTile),
                        endHeading)
                .build();
        queueAdd(newTrajectorySegment1);
        queueAdd(newTrajectorySegment2);

         */
    }
    public void update() {
        lastPose = drivetrain.getPoseEstimate();
    }
}

class returnThingy {
    public TileCalculation.Tile nextTile;
    public double startHeading;
    public double endHeading;
    public Pose2d startPose;
    public returnThingy(TileCalculation.Tile nextTile, double startHeading, double endHeading, Pose2d startPose) {
        this.nextTile = nextTile;
        this.startHeading = startHeading;
        this.endHeading = endHeading;
        this.startPose = startPose;
    }
}
