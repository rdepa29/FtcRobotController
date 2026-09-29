package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.drivetrain.Pose;

/**
 * Physical facts about the 2026-2027 field, BIOBUZZ, plus the poses our robot
 * starts autonomous from.
 *
 * <h2>Where each number came from</h2>
 *
 * <p>Every value is tagged as either CONFIRMED (published by FIRST in the
 * Competition Manual, Section 9) or MEASURE (you take it off the official field CAD
 * or your own robot with a tape measure). Do not let these two categories blur
 * together; that is how a "works on our practice field" bug reaches competition.
 *
 * <p>Source: FIRST Tech Challenge Competition Manual, BIOBUZZ, Section 9 (ARENA).
 * Current manual is Team Update 02, updated 24 Sep 2026.
 */
public class FieldConfig {

    // ==================================================================
    // FIELD
    // ==================================================================

    /** CONFIRMED. The field is 144in x 144in measured inside the perimeter walls. */
    public static final double FIELD_SIZE_INCHES = 144.0;

    /** CONFIRMED. 36 foam tiles, 6 columns by 6 rows, each 24in square. */
    public static final double TILE_SIZE_INCHES = 24.0;
    public static final int TILES_PER_SIDE = 6;

    // ==================================================================
    // TILE COORDINATES
    //
    // The manual defines columns A-F and rows 1-6, and states that red occupies
    // columns A, B, C and blue occupies D, E, F, viewed with red on the left
    // from the primary audience viewing direction. (Section 9.5, rule G304.)
    //
    // CONFIRMED: red loading zone is tile A5, red garden is tile A1,
    //            blue loading zone is tile F2, blue garden is tile F6.
    // MEASURE:  which physical end of the field is row 1 and which is row 6.
    //            The manual does not say. Read it off the official CAD before
    //            you trust any waypoint built on this.
    // ==================================================================

    public enum Tile {
        A1, A2, A3, A4, A5, A6,
        B1, B2, B3, B4, B5, B6,
        C1, C2, C3, C4, C5, C6,
        D1, D2, D3, D4, D5, D6,
        E1, E2, E3, E4, E5, E6,
        F1, F2, F3, F4, F5, F6
    }

    /**
     * Center of a tile, in inches, measured from the field's lower-left corner as
     * (0, 0) and increasing to the upper-right as (144, 144).
     *
     * <p>The origin corner and the row ordering are MEASURE items, see above.
     */
    public static double tileX(Tile tile) {
        int column = tile.name().charAt(0) - 'A';
        return (column + 0.5) * TILE_SIZE_INCHES;
    }

    public static double tileY(Tile tile) {
        int row = tile.name().charAt(1) - '1';
        return (row + 0.5) * TILE_SIZE_INCHES;
    }

    /**
     * Center of a tile as a {@link Pose}, so a waypoint can be written by name:
     *
     * <pre>
     *   drive.goTo(FieldConfig.cell(Tile.C3));
     *   drive.goTo(FieldConfig.cell(Tile.C3), 90);
     * </pre>
     *
     * <p>Heading comes from {@code heading} because "arrive at C3" does not imply a
     * facing. If the origin corner and row order are still MEASURE items, every
     * waypoint built on this is wrong the same way, so confirm them against the field
     * CAD before trusting a routine.
     */
    public static Pose cell(Tile tile) {
        return cell(tile, 0.0);
    }

    /** Center of a tile as a {@link Pose}, arriving with the given heading. */
    public static Pose cell(Tile tile, double headingDegrees) {
        return new Pose(tileX(tile), tileY(tile), headingDegrees);
    }

    /**
     * Center of a tile in a 1-based column and row, so a waypoint can be typed
     * numerically without importing the enum.
     *
     * @param column 1 for A, 2 for B, ... 6 for F
     * @param row 1 through 6
     */
    public static Pose cell(int column, int row) {
        return cell(Tile.valueOf("" + (char) ('A' + column - 1) + row), 0.0);
    }

    // ==================================================================
    // HIVE STRUCTURE  (center of the field, the main obstacle)
    // ==================================================================

    /** CONFIRMED. Base of the frame, which is also its widest point. */
    public static final double HIVE_FRAME_WIDTH_INCHES = 49.46;
    public static final double HIVE_FRAME_DEPTH_INCHES = 38.95;

    /** CONFIRMED. The hinge axis both HIVEs rotate around. */
    public static final double HIVE_PIVOT_HEIGHT_INCHES = 43.95;

    /** CONFIRMED. Distance between the two CELLs on a single HIVE. */
    public static final double HIVE_CELL_SPACING_INCHES = 18.8;

    /** CONFIRMED. The opening a robot has to launch a scoring element into. */
    public static final double CELL_OPENING_WIDTH_INCHES = 20.0;
    public static final double CELL_OPENING_HEIGHT_INCHES = 14.0;
    public static final double CELL_OPENING_DEPTH_INCHES = 12.0;

    /**
     * MEASURE. How high the up-facing CELL's opening sits above the tiles, and how
     * far the tipped CELL leans past vertical. This is what sets your shooter's
     * launch angle, and the manual only shows it in a figure.
     */
    public static final double CELL_OPENING_HEIGHT_MEASURE = 24.0;
    public static final double HIVE_TIP_ANGLE_MEASURE = 15.0;

    // ==================================================================
    // FLOWERS  (four, on the perimeter walls)
    // ==================================================================

    /** CONFIRMED. Top opening, where robots place scoring elements late in a match. */
    public static final double FLOWER_TOP_OPENING_DIAMETER_INCHES = 4.0;
    public static final double FLOWER_TOP_HEIGHT_INCHES = 21.5;

    /** CONFIRMED. Bottom retrieval opening, where robots collect pollen. */
    public static final double FLOWER_BOTTOM_OPENING_HEIGHT_INCHES = 3.55;
    public static final double FLOWER_BOTTOM_OPENING_DEPTH_INCHES = 3.57;

    /** MEASURE. Which corner of the field each of the four FLOWERS sits in. */
    public enum Flower {
        LOWER_LEFT, UPPER_LEFT, LOWER_RIGHT, UPPER_RIGHT
    }

    // ==================================================================
    // ZONES
    // ==================================================================

    /** CONFIRMED. The strip robots park in for the SWARM ranking point. */
    public static final double LOADING_ZONE_WIDTH_INCHES = 23.0;
    public static final double LOADING_ZONE_DEPTH_INCHES = 11.0;
    public static final Tile RED_LOADING_ZONE_TILE = Tile.A5;
    public static final Tile BLUE_LOADING_ZONE_TILE = Tile.F2;

    /** CONFIRMED. The thin strip along a wall where pollen scores. */
    public static final double GARDEN_WIDTH_INCHES = 23.0;
    public static final double GARDEN_DEPTH_INCHES = 2.0;
    public static final Tile RED_GARDEN_TILE = Tile.A1;
    public static final Tile BLUE_GARDEN_TILE = Tile.F6;

    // ==================================================================
    // ROBOT LIMITS  (Robot Construction Rules, R102 / R105)
    // ==================================================================

    /** CONFIRMED. The robot must fit entirely inside this cube at match start. */
    public static final double START_CUBE_INCHES = 18.0;

    /**
     * CONFIRMED. The expanded footprint the robot must always fit inside. Note that
     * 29in is the height above the field surface, not overall robot height, which is
     * how the drivetrain can grow outward during a match but not downward.
     */
    public static final double EXPANDED_WIDTH_INCHES = 18.0;
    public static final double EXPANDED_LENGTH_INCHES = 24.0;
    public static final double EXPANDED_HEIGHT_INCHES = 29.0;

    // ==================================================================
    // SCORING ELEMENTS  (Section 9.8)
    // ==================================================================

    /** CONFIRMED. Pollen are 2.8in yellow balls, alliance neutral. */
    public static final double POLLEN_DIAMETER_INCHES = 2.8;

    /** CONFIRMED. Nectar are 3.6in alliance-colored balls and noticeably heavier. */
    public static final double NECTAR_DIAMETER_INCHES = 3.6;

    /** CONFIRMED. Rule G304: robots may hold this many pollen at the start. */
    public static final int PRELOADED_POLLEN = 4;
}
