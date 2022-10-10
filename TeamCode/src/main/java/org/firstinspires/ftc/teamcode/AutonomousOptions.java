package org.firstinspires.ftc.teamcode;


public class AutonomousOptions {
    private static final int delayStartSeconds = 0;
    private AllianceColor alliance;
    private StartPosition startPosition;
    private ParkLocation parklocation;
    private ParkOnSignalZone parkOnSignalZone;
    private PlaceConeInTerminal placeConeInTerminal;
    private PlaceConesOnJunctions placeConesOnJunctions;

    public int getDelatStartSeconds() {
        return delayStartSeconds;
    }

    public AllianceColor getAlliance() {
        return alliance;
    }

    public StartPosition getStartPosition() {
        return startPosition;
    }

    public ParkLocation getParklocation() {
        return parklocation;
    }

    public ParkOnSignalZone getParkOnSignalZone() {
        return parkOnSignalZone;
    }

    public PlaceConeInTerminal getPlaceConeInTerminal() {
        return placeConeInTerminal;
    }

    public PlaceConesOnJunctions getPlaceConesOnJunctions() {
        return placeConesOnJunctions;
    }

    public enum AllianceColor {
        None,       //Make the driver select a color.
        Red,
        Blue
    }

    /*
     * Where do we start the robot
     * Right is on the right of your substation.
     * Left is on the left of your substation.
     */
    public enum StartPosition {
        None,
        Left,
        Right;
    }

    /*
     * Where do we park. Default is do not park.
     */
    public enum ParkLocation {
        None,
        Terminal,
        SubStation;

        public ParkLocation getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /*
     * Yes means park on the signal zone.
     * Default is No.
     */
    public enum ParkOnSignalZone {
        No,
        Yes;

        public ParkOnSignalZone getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /*
     * Yes means place cones on the junctions.
     * Default is No.
     */
    public enum PlaceConesOnJunctions {
        No,
        Yes;

        public PlaceConesOnJunctions getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /*
     * Yes means place cone in the terminal.
     *  No is the default.
     */
    public enum PlaceConeInTerminal {
        No,
        Yes;

        public PlaceConeInTerminal getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }
}
