package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Ron on 11/16/2016.
 * Modified: 9/14/2022
 * <p>
 * This class provides configuration for an autonomous opMode.
 * Most games benefit from autonomous opModes that can implement
 * different behavior based on an alliance strategy agreed upon
 * for a specific match.
 * </p>
 * <p>
 * Creating multiple opModes to meet this requirement results in duplicate
 * code and an environment that makes it too easy for a driver to
 * choose the wrong opMode "in the heat of battle."
 * </p>
 * <p>
 * This class is a way to solve these problems.
 * </p>
 */

public class AutonomousConfiguration {
    private AllianceColor alliance;
    private StartPosition startPosition;
    private ParkLocation parklocation;
    private ParkOnSignalZone parkOnSignalZone;
    private PlaceConesOnJunctions placeConesOnJunctions;
    private PlaceConeInTerminal placeConeInTerminal;
    private int delayStartSeconds;
    private boolean readyToStart;
    private Telemetry telemetry;
    private Telemetry.Item teleAlliance;
    private Telemetry.Item teleStartPosition;
    private Telemetry.Item teleParkLocation;
    private Telemetry.Item teleParkOnSignalZone;
    private Telemetry.Item telePlaceConeInTerminal;
    private Telemetry.Item telePlaceConesOnJunctions;
    private Telemetry.Item teleDelayStartSeconds;
    private Telemetry.Item teleReadyToStart;

    private DebouncedButton aButton;
    private DebouncedButton bButton;
    private DebouncedButton dPadLeft;
    private DebouncedButton dPadRight;
    private DebouncedButton dPadDown;
    private DebouncedButton dPadUp;
    private DebouncedButton leftBumper;
    private DebouncedButton rightBumper;
    private DebouncedButton startButton;
    private DebouncedButton xButton;
    private DebouncedButton yButton;
    private DebouncedButton leftStickButton;
    private DebouncedButton rightStickButton;


    /*
     * Pass in the gamepad and telemetry from your opMode.
     */
    public void init(Gamepad gamepad, Telemetry telemetry1) {
        NinjaGamePad gamePad1 = new NinjaGamePad(gamepad);
        aButton = gamePad1.getAButton().debounced();
        bButton = gamePad1.getBButton().debounced();
        dPadLeft = gamePad1.getDpadLeft().debounced();
        dPadRight = gamePad1.getDpadRight().debounced();
        dPadDown = gamePad1.getDpadDown().debounced();
        dPadUp = gamePad1.getDpadUp().debounced();
        rightBumper = gamePad1.getRightBumper().debounced();
        leftBumper = gamePad1.getLeftBumper().debounced();
        xButton = gamePad1.getXButton().debounced();
        yButton = gamePad1.getYButton().debounced();
        leftStickButton = gamePad1.getLeftStickButton().debounced();
        rightStickButton = gamePad1.getRightStickButton().debounced();
        startButton = gamePad1.getStartButton().debounced();
        this.telemetry = telemetry1;

        // Default selections if driver does not select anything.
        alliance = AllianceColor.None;
        startPosition = StartPosition.None;
        parklocation = ParkLocation.None;
        placeConesOnJunctions = PlaceConesOnJunctions.No;
        parkOnSignalZone = ParkOnSignalZone.No;
        placeConeInTerminal = PlaceConeInTerminal.No;
        readyToStart = false;
        delayStartSeconds = 0;

        ShowHelp();
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

    public PlaceConesOnJunctions getPlaceConesOnJunctions() {
        return placeConesOnJunctions;
    }

    public PlaceConeInTerminal getPlaceConeInTerminal() {
        return placeConeInTerminal;
    }

    public int getDelayStartSeconds() {
        return delayStartSeconds;
    }

    public boolean getReadyToStart() {
        return readyToStart;
    }

    private void ShowHelp() {
        teleAlliance = telemetry.addData("X = Blue, B = Red", getAlliance());
        teleStartPosition = telemetry.addData("D-pad left/right, select start position", getStartPosition());
        teleParkLocation = telemetry.addData("D-pad up to cycle park location", getParklocation());
        teleParkOnSignalZone = telemetry.addData("D-pad down to cycle park on signal zone", getParkOnSignalZone());
        telePlaceConesOnJunctions = telemetry.addData("Y to cycle cones on junctions", getPlaceConesOnJunctions());
        telePlaceConeInTerminal = telemetry.addData("A to cycle place cone in terminal", getPlaceConeInTerminal());
        teleDelayStartSeconds = telemetry.addData("Left & Right buttons, Delay Start", getDelayStartSeconds());
        teleReadyToStart = telemetry.addData("Ready to start: ", getReadyToStart());
    }

    // Call this in a loop from your opMode. It will returns true if you press the
    // game pad Start.
    public void init_loop() {
        //Alliance Color
        if (xButton.getRise()) {
            alliance = AllianceColor.Blue;
            telemetry.speak("blue");
        }

        if (bButton.getRise()) {
            alliance = AllianceColor.Red;
            telemetry.speak("red");
        }
        teleAlliance.setValue(alliance);

        //Start Position
        if (dPadRight.getRise()) {
            startPosition = StartPosition.Right;
            telemetry.speak("start right");
        }

        if (dPadLeft.getRise()) {
            startPosition = StartPosition.Left;
            telemetry.speak("start left");
        }
        teleStartPosition.setValue(startPosition);

        //Park Location
        if (dPadUp.getRise()) {
            parklocation = parklocation.getNext();
            switch (parklocation) {
                case None:
                    telemetry.speak("park, no.");
                    break;
                case SubStation:
                    telemetry.speak("park in substation");
                    break;
                case Terminal:
                    telemetry.speak("park in terminal");
                    break;
            }
        }
        teleParkLocation.setValue(parklocation);

        //Park on Signal Zone
        if (dPadDown.getRise()) {
            parkOnSignalZone = parkOnSignalZone.getNext();
            switch (parkOnSignalZone) {
                case Yes:
                    telemetry.speak("park on signal zone, yes");
                    break;
                case No:
                    telemetry.speak("park on signal zone,, no");
                    break;
            }
        }
        teleParkOnSignalZone.setValue(parkOnSignalZone);

        //Place cones on junction.
        if (yButton.getRise()) {
            placeConesOnJunctions = placeConesOnJunctions.getNext();
            switch (placeConesOnJunctions) {
                case Yes:
                    telemetry.speak("place cones on junctions, yes");
                    break;
                case No:
                    telemetry.speak("place cones on junctions, no");
                    break;
            }
        }
        telePlaceConesOnJunctions.setValue(placeConesOnJunctions);

        //Place cone in terminal
        if (aButton.getRise()) {
            placeConeInTerminal = placeConeInTerminal.getNext();
            switch (placeConeInTerminal) {
                case Yes:
                    telemetry.speak("place cone in terminal, yes");
                    break;
                case No:
                    telemetry.speak("place cone in terminal, no");
            }
        }
        telePlaceConeInTerminal.setValue(placeConeInTerminal);

        // Keep range within 0-15 seconds. Wrap at either end.
        if (leftBumper.getRise()) {
            delayStartSeconds = delayStartSeconds - 1;
            delayStartSeconds = (delayStartSeconds < 0) ? 15 : delayStartSeconds;
            telemetry.speak("delay start " + delayStartSeconds + " seconds");
        }
        if (rightBumper.getRise()) {
            delayStartSeconds = delayStartSeconds + 1;
            delayStartSeconds = (delayStartSeconds > 15) ? 0 : delayStartSeconds;
            telemetry.speak("delay start " + delayStartSeconds + " seconds");
        }
        teleDelayStartSeconds.setValue(delayStartSeconds);

        //Have the required options been set?
        readyToStart = alliance != AllianceColor.None && startPosition != StartPosition.None;
        teleReadyToStart.setValue(getReadyToStart());
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
