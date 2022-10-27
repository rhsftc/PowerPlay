package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Ron on 11/16/2016.
 * Modified: 10/12/2022
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
 * It is designed to used from opMode (iterative) class.
 * The selected options can also be saved to a file, allowing the
 * configuration options to be set before a match and to be available
 * to any op Mode.
 * </p>
 */

public class AutonomousConfiguration {
    private AutonomousOptions autonomousOptions;
    private Context context;
    private ReadWriteAutoOptions readWriteAutoOptions;
    private boolean readyToStart;
    private boolean savedToFile;
    private Telemetry telemetry;
    private Telemetry.Item teleAlliance;
    private Telemetry.Item teleStartPosition;
    private Telemetry.Item teleParkLocation;
    private Telemetry.Item teleParkOnSignalZone;
    private Telemetry.Item telePlaceConeInTerminal;
    private Telemetry.Item telePlaceConesOnJunctions;
    private Telemetry.Item teleDelayStartSeconds;
    private Telemetry.Item teleReadyToStart;
    private Telemetry.Item teleSavedToFile;

    private DebouncedButton aButton;
    private DebouncedButton bButton;
    private DebouncedButton dPadLeft;
    private DebouncedButton dPadRight;
    private DebouncedButton dPadDown;
    private DebouncedButton dPadUp;
    private DebouncedButton leftBumper;
    private DebouncedButton rightBumper;
    private DebouncedButton startButton;
    private DebouncedButton backButton;
    private DebouncedButton xButton;
    private DebouncedButton yButton;
    private DebouncedButton leftStickButton;
    private DebouncedButton rightStickButton;


    /*
     * Pass in the gamepad and telemetry from your opMode.
     */
    public void init(Gamepad gamepad, Telemetry telemetry1, Context context) {
        this.context = context;
        readWriteAutoOptions = new ReadWriteAutoOptions(context);
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
        backButton = gamePad1.getBackButton().debounced();
        //backButton=gamePad1.
        this.telemetry = telemetry1;
        // See if we saved the options yet. If not, save the defaults.
        autonomousOptions = new AutonomousOptions();
        if (!readWriteAutoOptions.optionsAreSaved()) {
            resetOptions();
            this.SaveOptions();
        } else {
            autonomousOptions = getSaveAutoOptions();
        }

        ShowHelp();
    }

    public AutonomousOptions.AllianceColor getAlliance() {
        return autonomousOptions.getAllianceColor();
    }

    public AutonomousOptions.StartPosition getStartPosition() {
        return autonomousOptions.getStartPosition();
    }

    public AutonomousOptions.ParkLocation getParklocation() {
        return autonomousOptions.getParklocation();
    }

    public AutonomousOptions.ParkOnSignalZone getParkOnSignalZone() {
        return autonomousOptions.getParkOnSignalZone();
    }

    public AutonomousOptions.PlaceConesOnJunctions getPlaceConesOnJunctions() {
        return autonomousOptions.getPlaceConesOnJunctions();
    }

    public AutonomousOptions.PlaceConeInTerminal getPlaceConeInTerminal() {
        return autonomousOptions.getPlaceConeInTerminal();
    }

    public int getDelayStartSeconds() {
        return autonomousOptions.getDelayStartSeconds();
    }

    public boolean getReadyToStart() {
        return readyToStart;
    }

    private void ShowHelp() {
        teleAlliance = telemetry.addData("X = Blue, B = Red", autonomousOptions.getAllianceColor());
        teleStartPosition = telemetry.addData("D-pad left/right, select start position", autonomousOptions.getStartPosition());
        teleParkLocation = telemetry.addData("D-pad up to cycle park location", autonomousOptions.getParklocation());
        teleParkOnSignalZone = telemetry.addData("D-pad down to cycle park on signal zone", autonomousOptions.getParkOnSignalZone());
        telePlaceConesOnJunctions = telemetry.addData("Y to cycle cones on junctions", autonomousOptions.getPlaceConesOnJunctions());
        telePlaceConeInTerminal = telemetry.addData("A to cycle place cone in terminal", autonomousOptions.getPlaceConeInTerminal());
        teleDelayStartSeconds = telemetry.addData("Left & Right buttons, Delay Start", autonomousOptions.getDelayStartSeconds());
        teleReadyToStart = telemetry.addData("Ready to start: ", getReadyToStart());
        teleSavedToFile = telemetry.addData("Saved to file:", savedToFile);
        telemetry.addLine("Back button resets all options.");
    }

    // Call this in the init_loop from your opMode. It will returns true if you press the
    // game pad Start.
    public void init_loop() {
        //Set default options (ignore what was saved to the file.)
        if (backButton.getRise()) {
            resetOptions();
        }
        //Alliance Color
        if (xButton.getRise()) {
            autonomousOptions.setAllianceColor(AutonomousOptions.AllianceColor.Blue);
            telemetry.speak("blue");
        }

        if (bButton.getRise()) {
            autonomousOptions.setAllianceColor(AutonomousOptions.AllianceColor.Red);
            telemetry.speak("red");
        }
        teleAlliance.setValue(autonomousOptions.getAllianceColor());

        //Start Position
        if (dPadRight.getRise()) {
            autonomousOptions.setStartPosition(AutonomousOptions.StartPosition.Right);
            telemetry.speak("start right");
        }

        if (dPadLeft.getRise()) {
            autonomousOptions.setStartPosition(AutonomousOptions.StartPosition.Left);
            telemetry.speak("start left");
        }
        teleStartPosition.setValue(autonomousOptions.getStartPosition());

        //Park Location
        if (dPadUp.getRise()) {
            AutonomousOptions.ParkLocation parkLocation = autonomousOptions.getParklocation().getNext();
            switch (parkLocation) {
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
            autonomousOptions.setParkLocation(parkLocation);
            teleParkLocation.setValue(parkLocation);
        }

        //Park on Signal Zone
        if (dPadDown.getRise()) {
            AutonomousOptions.ParkOnSignalZone parkSignalZone = autonomousOptions.getParkOnSignalZone().getNext();
            switch (parkSignalZone) {
                case Yes:
                    telemetry.speak("park on signal zone, yes");
                    break;
                case No:
                    telemetry.speak("park on signal zone,, no");
                    break;
            }
            autonomousOptions.setParkOnSignalZone(parkSignalZone);
            teleParkOnSignalZone.setValue(parkSignalZone);
        }

        //Place cones on junction.
        if (yButton.getRise()) {
            AutonomousOptions.PlaceConesOnJunctions placeOnJunction = autonomousOptions.getPlaceConesOnJunctions().getNext();
            switch (placeOnJunction) {
                case Yes:
                    telemetry.speak("place cones on junctions, yes");
                    break;
                case No:
                    telemetry.speak("place cones on junctions, no");
                    break;
            }
            autonomousOptions.setPlaceConesOnJunctions(placeOnJunction);
            telePlaceConesOnJunctions.setValue(placeOnJunction);
        }

        //Place cone in terminal
        if (aButton.getRise()) {
            AutonomousOptions.PlaceConeInTerminal placeInTerminal = autonomousOptions.getPlaceConeInTerminal().getNext();
            switch (placeInTerminal) {
                case Yes:
                    telemetry.speak("place cone in terminal, yes");
                    break;
                case No:
                    telemetry.speak("place cone in terminal, no");
            }
            autonomousOptions.setPlaceConeInTerminal(placeInTerminal);
            telePlaceConeInTerminal.setValue(placeInTerminal);
        }

        // Keep range within 0-15 seconds. Wrap at either end.
        if (leftBumper.getRise()) {
            autonomousOptions.setDelayStartSeconds(autonomousOptions.getDelayStartSeconds() - 1);
            autonomousOptions.setDelayStartSeconds((autonomousOptions.getDelayStartSeconds() < 0) ? 15 : autonomousOptions.getDelayStartSeconds());
            telemetry.speak("delay start " + autonomousOptions.getDelayStartSeconds() + " seconds");
        }
        if (rightBumper.getRise()) {
            autonomousOptions.setDelayStartSeconds(autonomousOptions.getDelayStartSeconds() + 1);
            autonomousOptions.setDelayStartSeconds((autonomousOptions.getDelayStartSeconds() > 15) ? 0 : autonomousOptions.getDelayStartSeconds());
            telemetry.speak("delay start " + autonomousOptions.getDelayStartSeconds() + " seconds");
        }
        teleDelayStartSeconds.setValue(autonomousOptions.getDelayStartSeconds());

        //Have the required options been set?
        readyToStart = !(autonomousOptions.getAllianceColor() == AutonomousOptions.AllianceColor.None || autonomousOptions.getStartPosition() == AutonomousOptions.StartPosition.None);
        teleReadyToStart.setValue(readyToStart);

        //Save the options to a file if ready to start and start button is pressed.
        if (startButton.getRise() && getReadyToStart()) {
            SaveOptions();
            savedToFile = true;
            teleSavedToFile.setValue(true);
        }
    }

    // Default selections if driver does not select anything.
    private void resetOptions() {
        autonomousOptions.setAllianceColor(AutonomousOptions.AllianceColor.None);
        autonomousOptions.setStartPosition(AutonomousOptions.StartPosition.None);
        autonomousOptions.setParkLocation(AutonomousOptions.ParkLocation.None);
        autonomousOptions.setPlaceConeInTerminal(AutonomousOptions.PlaceConeInTerminal.No);
        autonomousOptions.setPlaceConesOnJunctions(AutonomousOptions.PlaceConesOnJunctions.No);
        autonomousOptions.setParkOnSignalZone(AutonomousOptions.ParkOnSignalZone.No);
        autonomousOptions.setDelayStartSeconds(0);
        readyToStart = false;
        savedToFile = false;
    }

    private void SaveOptions() {
        ReadWriteAutoOptions readWriteAutoOptions = new ReadWriteAutoOptions(context);
        readWriteAutoOptions.storeObject(autonomousOptions);
    }

    public AutonomousOptions getSaveAutoOptions() {
        ReadWriteAutoOptions readWriteAutoOptions = new ReadWriteAutoOptions(context);
        AutonomousOptions temp = readWriteAutoOptions.getObject();
        telemetry.addData("Start: ", temp.getStartPosition());
        telemetry.update();
        return temp;
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
        Right
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
