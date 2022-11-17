/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is an example of a linear op mode using the AutonomousConfiguration classs to select
 * various tasks for autonomous.
 * Not the use in a linear op mode is slightly different than an interative op mode.
 */

@Autonomous(name = "Options Menu Linear", group = "Autonomous", preselectTeleOp = "Color")
//@Disabled
public class RHSConfigMenuLinear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        AutonomousConfiguration autonomousConfiguration = new AutonomousConfiguration();
        autonomousConfiguration.init(this.gamepad1, this.telemetry, hardwareMap.appContext);

        while (!opModeIsActive()) {
            autonomousConfiguration.init_loop();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Alliance",autonomousConfiguration.getAlliance());
            telemetry.addData("Start Position",autonomousConfiguration.getStartPosition());
            telemetry.addData("Park Location",autonomousConfiguration.getParkLocation());
            telemetry.addData("Park on Signal Zone",autonomousConfiguration.getParkOnSignalZone());
            telemetry.addData("Place Cone in Terminal",autonomousConfiguration.getPlaceConeInTerminal());
            telemetry.addData("Place Cones on Junction",autonomousConfiguration.getPlaceConesOnJunctions());
            telemetry.addData("Delay Start",autonomousConfiguration.getDelayStartSeconds());
            telemetry.addData("Ready to Start",autonomousConfiguration.getReadyToStart());
            telemetry.update();
        }
    }
}
