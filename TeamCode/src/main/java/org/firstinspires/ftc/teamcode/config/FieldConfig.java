package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.drivetrain.Pose;

// biobuzz field numbers, every value tagged confirmed or measure so the two
// never get confused
//
// https://ftc-resources.firstinspires.org/ftc/game/manual
public class FieldConfig {

    // field

    // confirmed, 36 tiles, 6 by 6, 24in square
    public static final double TILE_SIZE_INCHES = 24.0;
    public static final int TILES_PER_SIDE = 6;

    // tile coordinates
        int row = tile.name().charAt(1) - '1';
        return (row + 0.5) * TILE_SIZE_INCHES;
    }

    // tile center as a pose so waypoints can be written by name, examples in
    // readme.md
    }

    // tile center as a pose, arriving with the given heading
    public static Pose cell(Tile tile, double headingDegrees) {
        return new Pose(tileX(tile), tileY(tile), headingDegrees);
    }

    // same thing from a 1-based column and row
    public static Pose cell(int column, int row) {
        return cell(Tile.valueOf("" + (char) ('A' + column - 1) + row), 0.0);
    }

    // hive structure, the main obstacle
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

    // measure, height of the up-facing cell opening and how far the tipped cell
    // leans past vertical, this is what sets the launch angle
    public static final double CELL_OPENING_HEIGHT_MEASURE = 24.0;
    public static final double HIVE_TIP_ANGLE_MEASURE = 15.0;

    // flowers, four, on the walls
    public static final double FLOWER_TOP_HEIGHT_INCHES = 21.5;

    // confirmed, bottom opening, pollen comes out here
    public static final double FLOWER_BOTTOM_OPENING_HEIGHT_INCHES = 3.55;
    public static final double FLOWER_BOTTOM_OPENING_DEPTH_INCHES = 3.57;

    // measure, which corner each flower sits in
    public enum Flower {
        LOWER_LEFT, UPPER_LEFT, LOWER_RIGHT, UPPER_RIGHT
    }

    // zones
    public static final double LOADING_ZONE_WIDTH_INCHES = 23.0;
    public static final double LOADING_ZONE_DEPTH_INCHES = 11.0;
    public static final Tile RED_LOADING_ZONE_TILE = Tile.A5;
    public static final Tile BLUE_LOADING_ZONE_TILE = Tile.F2;

    // confirmed, thin strip along a wall where pollen scores
    public static final double GARDEN_WIDTH_INCHES = 23.0;
    public static final double GARDEN_DEPTH_INCHES = 2.0;
    public static final Tile RED_GARDEN_TILE = Tile.A1;
    public static final Tile BLUE_GARDEN_TILE = Tile.F6;

    // robot limits, R102 / R105
    public static final double START_CUBE_INCHES = 18.0;

    // confirmed, footprint it must always fit, 29in is height above the field
    // and not robot height, so the drivetrain can grow outward but not downward
    public static final double EXPANDED_WIDTH_INCHES = 18.0;
    public static final double EXPANDED_LENGTH_INCHES = 24.0;
    public static final double EXPANDED_HEIGHT_INCHES = 29.0;

    // scoring elements, section 9.8
    public static final double POLLEN_DIAMETER_INCHES = 2.8;

    // confirmed, nectar are 3.6in alliance-colored and heavier
    public static final double NECTAR_DIAMETER_INCHES = 3.6;

    // confirmed, rule G304, pollen you may preload
    public static final int PRELOADED_POLLEN = 4;
}
