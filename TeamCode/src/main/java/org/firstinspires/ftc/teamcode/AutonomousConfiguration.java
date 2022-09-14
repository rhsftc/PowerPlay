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
    private boolean gamepad1IsOK, gamepad2IsOK;
    private AllianceColor alliance;
    private StartPosition startPosition;
    private ParkLocation parklocation;
    private DeliverDuck deliverDuck;
    private DeliverFreight deliverFreight;
    private int delayStartSeconds;
    private NinjaGamePad gamePad1;
    private Telemetry telemetry;
    private Telemetry.Item teleAlliance;
    private Telemetry.Item teleStartPosition;
    private Telemetry.Item teleParkLocation;
    private Telemetry.Item teleDeliverDuck;
    private Telemetry.Item teleDeliverFreight;
    private Telemetry.Item teleDelayStartSeconds;
    private DebouncedButton aButton;
    private DebouncedButton bButton;
    private DebouncedButton dPadLeft;
    private DebouncedButton dPadRight;
    private DebouncedButton dPadDown;
    private DebouncedButton dPadUp;
    private DebouncedButton leftBumper;
    private DebouncedButton rightBumper;
    private DebouncedButton leftStickButton;
    private DebouncedButton rightStickButton;
    private DebouncedButton startButton;
    private DebouncedButton xButton;
    private DebouncedButton yButton;

    /*
     * Pass in gamepad and telemetry from your opMode when creating this object.
     */
    public AutonomousConfiguration(Gamepad gamepad, Telemetry telemetry1) {
        gamepad1IsOK = false;
        gamepad2IsOK = false;
        this.gamePad1 = new NinjaGamePad(gamepad);
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
        alliance = AllianceColor.Red;
        startPosition = StartPosition.Front;
        parklocation = ParkLocation.None;
        deliverFreight = DeliverFreight.No;
        deliverDuck = DeliverDuck.No;
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

    public DeliverDuck getDeliverDuck() {
        return deliverDuck;
    }

    public DeliverFreight getDeliverFreight() {
        return deliverFreight;
    }

    public int DelayStartSeconds() {
        return delayStartSeconds;
    }

    private void ShowHelp() {
        teleAlliance = telemetry.addData("X = Blue, B = Red", getAlliance());
        teleStartPosition = telemetry.addData("D-pad left/right, select start position", getStartPosition());
        teleParkLocation = telemetry.addData("D-pad up to cycle park location", getParklocation());
        teleDeliverDuck = telemetry.addData("D-pad down to cycle deliver duck", getDeliverDuck());
        teleDeliverFreight = telemetry.addData("Y to cycle deliver freight", getDeliverFreight());
        teleDelayStartSeconds = telemetry.addData("Left & Right buttons, Delay Start", DelayStartSeconds());
        telemetry.addLine("Game pad or app Start will end selection.");
    }

    // Call this in a loop from your opMode. It will returns true if you press the
    // game pad Start.
    public boolean GetOptions() {
        if (xButton.getRise()) {
            alliance = AllianceColor.Blue;
            telemetry.speak("blue");
        }

        if (bButton.getRise()) {
            alliance = AllianceColor.Red;
            telemetry.speak("red");
        }

        teleAlliance.setValue(alliance);

        if (dPadLeft.getRise()) {
            startPosition = StartPosition.Back;
            telemetry.speak("start back");
        }

        if (dPadRight.getRise()) {
            startPosition = StartPosition.Front;
            telemetry.speak("start front");
        }

        teleStartPosition.setValue(startPosition);

        if (dPadUp.getRise()) {
            parklocation = parklocation.getNext();
            switch (parklocation) {
                case StorageUnit:
                    telemetry.speak("park in storage unit");
                    break;
                case WarehouseFront:
                    telemetry.speak("park in warehouse front");
                    break;
                case WarehouseBack:
                    telemetry.speak("park in warehouse back");
                    break;
            }
        }

        teleParkLocation.setValue(parklocation);

        if (dPadDown.getRise()) {
            deliverDuck = deliverDuck.getNext();
            switch (deliverDuck) {
                case Yes:
                    telemetry.speak("deliver duck, yes");
                    break;
                case No:
                    telemetry.speak("deleiver duck, no");
                    break;
            }
        }

        teleDeliverDuck.setValue(deliverDuck);

        if (yButton.getRise()) {
            deliverFreight = deliverFreight.getNext();
            switch (deliverFreight) {
                case Yes:
                    telemetry.speak("deliver freight, yes");
                    break;
                case No:
                    telemetry.speak("deleiver freight, no");
                    break;
            }
        }

        teleDeliverFreight.setValue(deliverFreight);

        // Keep range within 0-15 seconds. Wrap at either end.
        if (leftBumper.getRise()) {
            delayStartSeconds = delayStartSeconds - 1;
            delayStartSeconds = (delayStartSeconds < 0) ? delayStartSeconds = 15 : delayStartSeconds;
            telemetry.speak("delay start " + delayStartSeconds + " seconds");
        }
        if (rightBumper.getRise()) {
            delayStartSeconds = delayStartSeconds + 1;
            delayStartSeconds = (delayStartSeconds > 15) ? delayStartSeconds = 0 : delayStartSeconds;
            telemetry.speak("delay start " + delayStartSeconds + " seconds");
        }

        teleDelayStartSeconds.setValue(delayStartSeconds);

        if (startButton.getFall()) {
            telemetry.speak("ready to start");
            return true;
        }

        return false;
    }

    public enum AllianceColor {
        Red,
        Blue
    }

    /*
     * Where do we start the robot
     * Back is towards the warehouse.
     * Front if towards the audience.
     */
    public enum StartPosition {
        None,
        Back,
        Front;

        public StartPosition getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /*
     * Where do we park. Default is do not park.
     */
    public enum ParkLocation {
        None,
        WarehouseFront,
        WarehouseBack,
        StorageUnit;

        public ParkLocation getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /*
     * Yes means deliver the duck from the carousel.
     * Default is No.
     */
    public enum DeliverDuck {
        No,
        Yes;

        public DeliverDuck getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /*
     * Yes means deliver freight to the shipping hub.
     * Default is No.
     */
    public enum DeliverFreight {
        No,
        Yes;

        public DeliverFreight getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }
}
