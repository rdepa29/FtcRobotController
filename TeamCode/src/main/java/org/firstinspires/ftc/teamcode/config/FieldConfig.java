package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.drivetrain.Pose;

// biobuzz field numbers, each tagged confirmed or measure so the two never blur
// https://ftc-resources.firstinspires.org/ftc/game/manual
public class FieldConfig {

    /// field
    // confirmed, 144in square measured inside the walls
    public static final double FIELD_SIZE_INCHES = 144.0;

    // 36 tiles 6 by 6, 24in square
    public static final double TILE_SIZE_INCHES = 24.0;
    public static final int TILES_PER_SIDE = 6;

    // tile coordinates
    public enum Tile {
        A1, A2, A3, A4, A5, A6,
        B1, B2, B3, B4, B5, B6,
        C1, C2, C3, C4, C5, C6,
        D1, D2, D3, D4, D5, D6,
        E1, E2, E3, E4, E5, E6,
        F1, F2, F3, F4, F5, F6
    }

    // tile center in inches, (0,0) lower-left to (144,144) upper-right
    public static double tileX(Tile tile) {
        int column = tile.name().charAt(0) - 'A';
        return (column + 0.5) * TILE_SIZE_INCHES;
    }

    public static double tileY(Tile tile) {
        int row = tile.name().charAt(1) - '1';
        return (row + 0.5) * TILE_SIZE_INCHES;
    }

    // tile center as a pose so waypoints can be written by name
    public static Pose cell(Tile tile) {
        return cell(tile, 0.0);
    }

    // tile center as a pose, arriving with the given heading
    public static Pose cell(Tile tile, double headingDegrees) {
        return new Pose(tileX(tile), tileY(tile), headingDegrees);
    }

    // same thing from a 1-based column and row, column: 1 for A, 2 for B, ... 6 for F
    // row: 1 through 6
    public static Pose cell(int column, int row) {
        return cell(Tile.valueOf("" + (char) ('A' + column - 1) + row), 0.0);
    }

    /// hive structure, the main obstacle
    // confirmed, base of the frame and its widest point
    public static final double HIVE_FRAME_WIDTH_INCHES = 49.46;
    public static final double HIVE_FRAME_DEPTH_INCHES = 38.95;

    // confirmed, hinge axis both hives rotate around
    public static final double HIVE_PIVOT_HEIGHT_INCHES = 43.95;

    // confirmed, distance between the two cells on one hive
    public static final double HIVE_CELL_SPACING_INCHES = 18.8;

    // confirmed, opening you launch a scoring element into
    public static final double CELL_OPENING_WIDTH_INCHES = 20.0;
    public static final double CELL_OPENING_HEIGHT_INCHES = 14.0;
    public static final double CELL_OPENING_DEPTH_INCHES = 12.0;

    // measure, cell opening height and tip angle set the launch angle
    public static final double CELL_OPENING_HEIGHT_MEASURE = 24.0;
    public static final double HIVE_TIP_ANGLE_MEASURE = 15.0;

    /// flowers, four, on the walls
    // confirmed, top opening, scoring elements go in late
    public static final double FLOWER_TOP_OPENING_DIAMETER_INCHES = 4.0;
    public static final double FLOWER_TOP_HEIGHT_INCHES = 21.5;

    // confirmed, bottom opening, pollen comes out here
    public static final double FLOWER_BOTTOM_OPENING_HEIGHT_INCHES = 3.55;
    public static final double FLOWER_BOTTOM_OPENING_DEPTH_INCHES = 3.57;

    // measure, which corner each flower sits in
    public enum Flower {
        LOWER_LEFT, UPPER_LEFT, LOWER_RIGHT, UPPER_RIGHT
    }

    /// zones
    // confirmed, strip you park in for swarm
    public static final double LOADING_ZONE_WIDTH_INCHES = 23.0;
    public static final double LOADING_ZONE_DEPTH_INCHES = 11.0;
    public static final Tile RED_LOADING_ZONE_TILE = Tile.A5;
    public static final Tile BLUE_LOADING_ZONE_TILE = Tile.F2;

    // confirmed, thin strip along the wall where pollen scores
    public static final double GARDEN_WIDTH_INCHES = 23.0;
    public static final double GARDEN_DEPTH_INCHES = 2.0;
    public static final Tile RED_GARDEN_TILE = Tile.A1;
    public static final Tile BLUE_GARDEN_TILE = Tile.F6;

    /// robot limits, R102 / R105
    // confirmed, cube the robot must fit inside at match start
    public static final double START_CUBE_INCHES = 18.0;

    // confirmed, footprint it must always fit, 29in is field height not robot
    public static final double EXPANDED_WIDTH_INCHES = 18.0;
    public static final double EXPANDED_LENGTH_INCHES = 24.0;
    public static final double EXPANDED_HEIGHT_INCHES = 29.0;

    /// scoring elements, section 9.8
    // confirmed, pollen are 2.8in yellow balls, alliance neutral
    public static final double POLLEN_DIAMETER_INCHES = 2.8;

    // confirmed, nectar are 3.6in alliance-colored and heavier
    public static final double NECTAR_DIAMETER_INCHES = 3.6;

    // confirmed, rule G304, pollen you may preload
    public static final int PRELOADED_POLLEN = 4;
}
