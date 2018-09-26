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

import android.content.Context;
import android.content.pm.PackageManager;
//import android.graphics.Camera;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.hardware.Camera;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;

import java.nio.channels.Pipe;
import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import static java.lang.Thread.sleep;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="BasicOpMode_Simple_Camera", group="Iterative Opmode")
//@Disabled
public class BasicOpMode_Simple_Camera extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Frame image;
    int frame_count = 1;
    int empty_count = 1;
    int error_count = 1;
    ReadWriteLock lock = new ReentrantReadWriteLock();
    int count = 0;


    //private FullGoldPipe
    VuforiaLocalizer vuforia;



    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName;
    BlockingQueue<VuforiaLocalizer.CloseableFrame> imageQueue;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialization Started");



        /*
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AYVsAlX/////AAABmWbkdZ4I80lPnylHGLs3cnlCLb1qe8c1ZLUAyTfbRS" +
                "OEMWD92vp4FGQ5SUHIZX+8ae9AhFL1OBIQfkhzcC4YSbs+6vMXwbEHwv+8BgquAUVkVmOXNEKibv/qvuk" +
                "tOkNu88hX1FVe9Lprh/XHNRBjvAKTC2Hu+x/dcZrrkT2erpxXgkOY5dDgUhkbGoCvhXqL7tNKku65hmiz" +
                "u0NZ2NbqnTCfw4Iev3dUeagzTk2mvq9SoFQZRo1FnOoZ/tiy1TMimz8V+06F6QKV9rknfqteQ//FaRqKT/" +
                "t6V/FE6vUOI11mZGo79rIl7XCCmeUwqMrik38gqHyMFzO3lzXLeiC75FgFGmlPmT6FHjSp5rmWGTNM";


         * We also indicate which camera on the RC we wish to use.



        parameters.cameraName = webcamName;


        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "Aa7bhCH/////AAABmS9BhZBYq0q8pHVTi1/FrUqOqb71oAHO5npGz9N1KSKNOAkG/FwvWv7QCsz7epKVLmwhuekWxfGPjI9e9S8qmTA4MpK2ArzKE7dQGNJiOm4+wAtBnycortCdjRvzeU2DglqA8LZFqj0cN9blx40X0VWG+LkoJE0TPoeP537SsImqvZPeqWVd23nG5rmInJ/L0jObMZZl5SwQLJDPySqwfVZmxkblLUYIlCcHLNfOZhAfUYnynPTuOe9m9n6w7DN8j61ErKG7WcuwnTVQCZJ9S+6COgCG6IpTWiPfKDnoJe1V8GDmydUFSVIqGCUKUfPcepo2NxUCkeP4FDSWJ8jivwr3DFCdQr9+RYpzlRsixyU4";

        ;

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        /**
         * Instantiate the Vuforia engine
         */
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        vuforia.enableConvertFrameToBitmap();

        vuforia.setFrameQueueCapacity(1);

        imageQueue = vuforia.getFrameQueue();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
        vuforia_thread.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        /*
        if (!(vuforia_thread.isAlive())) {
            vuforia_thread.start();
        }
        */


        lock.readLock().lock();
        try {
            //process(image);
            telemetry.addData("Frame Read", "\"processing\" frame");
        }
        finally {
            lock.readLock().unlock();
        }

        /*
        if (runtime.seconds() > 10){
            try {
                vuforia_thread.stop();
                telemetry.addData("Thread", "Stopped without Exception");
            }
            catch (Exception e){
                telemetry.addData("Thread", "Stopped with Exception");
            }
            runtime.reset();
        }
        */


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    Thread vuforia_thread = new Thread(new Runnable() {



        public synchronized void run() {
            while (true) {


                imageQueue = vuforia.getFrameQueue();
                if (!imageQueue.isEmpty()) {
                    lock.writeLock().lock();
                    try {

                        image = imageQueue.take();
                        telemetry.addData("Frame Write", "Got Frame: " + frame_count);
                        Log.d("Frame Write", "Got Grame");
                        frame_count = frame_count + 1;

                    } catch (Exception e) {
                        telemetry.addData("Frame Write", "Error" + error_count);
                        error_count = error_count + 1;
                        Log.d("Frame Write", "Error");
                    } finally {
                        lock.writeLock().unlock();
                    }
                } else {
                    telemetry.addData("Frame Write", "Frame is empty: " + empty_count);
                    Log.d("Frame Write", "Frame is Empty");
                    empty_count = empty_count + 1;
                }
            }


        }
    });




}
