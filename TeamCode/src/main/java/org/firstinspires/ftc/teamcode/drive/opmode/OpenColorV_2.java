
package org.firstinspires.ftc.teamcode.drive.opmode;/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//@TeleOp(name = "OpenCvRed")

public class OpenColorV_2
{

    public enum SkystonePosition
    {
        RED,//james
        BLUE,
        GREEN,
    }
    OpenCvWebcam webcam;
    SamplePipeline pipeline;
    static Telemetry telemetry;

    //@Override
    public void OpenCv(HardwareMap hardwareMap, Telemetry telemetryIn, String thisColor)
    {
        telemetry = telemetryIn;

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        pipeline = new SamplePipeline();
       // pipeline.setColor(thisColor);
        webcam.setPipeline(pipeline);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        //waitForStart();

        //while (opModeIsActive())
        //{

        //}

    }
/*
    public int analysis()
    {
        return pipeline.getAnalysis().ordinal();
    }
*/
    public void runWhileActive()
    {
        /*
         * Send some stats to the telemetry
         */

        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
      //  telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Max", pipeline.getMax());
        telemetry.update();
        //linearOpMode.sleep(50);

        /*
         * NOTE: stopping the stream from the camera early (before the end of the OpMode
         * when it will be automatically stopped for you) *IS* supported. The "if" statement
         * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
         */
        //if(gamepad1.a)
        //{
        /*
         * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
         * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
         * if the reason you wish to stop the stream early is to switch use of the camera
         * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
         * (commented out below), because according to the Android Camera API documentation:
         *         "Your application should only have one Camera object active at a time for
         *          a particular hardware camera."
         *
         * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
         * but it doesn't hurt to call it anyway, if for no other reason than clarity.
         *
         * NB2: if you are stopping the camera stream to simply save some processing power
         * (or battery power) for a short while when you do not need your vision pipeline,
         * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
         * it the next time you wish to activate your vision pipeline, which can take a bit of
         * time. Of course, this comment is irrelevant in light of the use case described in
         * the above "important note".
         */
        //    webcam.stopStreaming();
        //webcam.closeCameraDevice();
        //}


        /*
         * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
         * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
         * anyway). Of course in a real OpMode you will likely not want to do this.
         */
        //linearOpMode.sleep(100);
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    public static class SamplePipeline extends OpenCvPipeline
    {
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);
       // private volatile SkystonePosition position = SkystonePosition.LEFT;
        int cNum =0;
        int cNum1 =1;
        int cNum2 =2;
        static int posNum = 590;
        String colorBlue = "BLUE";
        String colorRed = "RED";
        String colorRedNeg = "REDNEG";



        Mat region2_Cb, region3_Cb,region4_Cb;
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        Mat Cb = new Mat();

        int avg2, avg3, avg4;
        static final int REGION_WIDTH = (250/1280)*640;
        static final int REGION_HEIGHT = (210/720)*480;

        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(newResX(256),newResY(250));

        public void setColor(String thisColor){

            if(thisColor.equals(colorBlue)){
                cNum = 2;
                posNum = 640;
            } else if (thisColor.equals(colorRed)){
                cNum = 1;
                posNum = 512;
            }
            //REGION3_TOPLEFT_ANCHOR_POINT = new Point((posNum/1280)*640,(250/720)*480);
            telemetry.addLine("Amongus");
            telemetry.update();
        }


        public static double newResX(double oldRes){
            double newRes;
            newRes = (oldRes/1280)*640;
            return newRes;
        }
        public static double newResY(double oldRes){
            double newRes;
            newRes = (oldRes/720)*480;
            return newRes;
        }

        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, cNum2);
        }
        void inputToCr(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, cNum1);
        }


        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);



        @Override
        public void init(Mat firstFrame)
        {
            inputToCr(firstFrame);
            region2_Cb = Cr.submat(new Rect(region2_pointA, region2_pointB));
            region4_Cb = Cr.submat(new Rect(region2_pointA, region2_pointB));
            inputToCb(firstFrame);
            region3_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));

        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCr(input);
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg4 = (int) Core.mean(region2_Cb).val[0];
            inputToCb(input);
            avg3 = (int) Core.mean(region2_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    20/1280*640); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    20/1280*640); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    20/1280*640); // Thickness of the rectangle lines


            int maxTwoThree = Math.max(avg3, avg2);
            int max = Math.max(maxTwoThree, avg4);

             if(max == avg2) // Was it from region 2?
            {
               // position = SkystonePosition.LEFT; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg3) // Was it from region 3?
            {
         //       position = SkystonePosition.CENTER; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg4) // Was it from region 2?
            {
        //        position = SkystonePosition.RIGHT; // Record our analysis

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }


            return input;
        }

      /*  public SkystonePosition getAnalysis()
        {
            return position;
        }
*/
        public int getMax(){
            int maxTwoThree = Math.max(avg3, avg2);
            int max = Math.max(maxTwoThree, avg4);

            return max;
        }

    }
}
