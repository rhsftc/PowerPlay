package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Ron on 11/16/2016.
 * Modified: 10/31/2022
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
    private GamepadEx gamepadEx;
    private Context context;
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

    /*
     * Pass in the gamepad and telemetry from your opMode.
     */
    public void init(Gamepad gamepad, Telemetry telemetry1, Context context) {
        this.context = context;
        ReadWriteAutoOptions readWriteAutoOptions = new ReadWriteAutoOptions(context);
        gamepadEx = new GamepadEx(gamepad);
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
        gamepadEx.readButtons();
        //Set default options (ignore what was saved to the file.)
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.BACK)) {
            resetOptions();
        }
        //Alliance Color
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.X)) {
            autonomousOptions.setAllianceColor(AutonomousOptions.AllianceColor.Blue);
            telemetry.speak("blue");
        }

        if (gamepadEx.wasJustReleased(GamepadKeys.Button.B)) {
            autonomousOptions.setAllianceColor(AutonomousOptions.AllianceColor.Red);
            telemetry.speak("red");
        }
        teleAlliance.setValue(autonomousOptions.getAllianceColor());

        //Start Position
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
            autonomousOptions.setStartPosition(AutonomousOptions.StartPosition.Right);
            telemetry.speak("start right");
        }

        if (gamepadEx.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
            autonomousOptions.setStartPosition(AutonomousOptions.StartPosition.Left);
            telemetry.speak("start left");
        }
        teleStartPosition.setValue(autonomousOptions.getStartPosition());

        //Park Location
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
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
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
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
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.Y)) {
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
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.A)) {
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
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
            autonomousOptions.setDelayStartSeconds(autonomousOptions.getDelayStartSeconds() - 1);
            autonomousOptions.setDelayStartSeconds((autonomousOptions.getDelayStartSeconds() < 0) ? 15 : autonomousOptions.getDelayStartSeconds());
            telemetry.speak("delay start " + autonomousOptions.getDelayStartSeconds() + " seconds");
        }
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)) {
            autonomousOptions.setDelayStartSeconds(autonomousOptions.getDelayStartSeconds() + 1);
            autonomousOptions.setDelayStartSeconds((autonomousOptions.getDelayStartSeconds() > 15) ? 0 : autonomousOptions.getDelayStartSeconds());
            telemetry.speak("delay start " + autonomousOptions.getDelayStartSeconds() + " seconds");
        }
        teleDelayStartSeconds.setValue(autonomousOptions.getDelayStartSeconds());

        //Have the required options been set?
        readyToStart = !(autonomousOptions.getAllianceColor() == AutonomousOptions.AllianceColor.None || autonomousOptions.getStartPosition() == AutonomousOptions.StartPosition.None);
        teleReadyToStart.setValue(readyToStart);

        //Save the options to a file if ready to start and start button is pressed.
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.START) && getReadyToStart()) {
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
}
