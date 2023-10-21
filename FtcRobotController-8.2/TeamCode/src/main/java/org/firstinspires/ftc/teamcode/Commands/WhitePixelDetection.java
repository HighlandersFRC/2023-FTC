package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public abstract class WhitePixelDetection extends Command{
    public Servo LServo;
    public Servo RServo;
    Boolean Done = false;
    public WhitePixelDetection(HardwareMap hardwareMap){
        WebcamName WebcamName = (hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "webcam1"));
    }
    public void start(){
        private void initTfod() {

            // Create the TensorFlow processor by using a builder.
            tfod = new TfodProcessor.Builder()

                    .setModelAssetName(TFOD_MODEL_ASSET)
                    // .setModelFileName(TFOD_MODEL_ASSET) //if you have downloaded a custom team model to the Robot Controller.
                    //.setModelAssetName(TFOD_MODEL_ASSET)
                    //.setModelFileName(TFOD_MODEL_FILE)
                    .setModelLabels(LABELS)
                    //.setModelLabels(LABELS)
                    //.setIsModelTensorFlow2(true)
                    //.setIsModelQuantized(true)
                    //.setModelInputSize(300)
                    //.setModelAspectRatio(16.0 / 9.0)

                    .build();

            // Create the vision portal by using a builder.
            VisionPortal.Builder builder = new VisionPortal.Builder();

            // Set the camera (webcam vs. built-in RC phone camera).
            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "webcam1"));
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
            }

            // Choose a camera resolution. Not all cameras support all resolutions.
            //builder.setCameraResolution(new Size(640, 480));

            // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
            builder.enableLiveView(true);

            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

            // Choose whether or not LiveView stops if no processors are enabled.
            // If set "true", monitor shows solid orange screen if no processors enabled.
            // If set "false", monitor shows camera view without annotations.
            //builder.setAutoStopLiveView(false);

            // Set and enable the processor.
            builder.addProcessor(tfod);

            // Build the Vision Portal, using the above settings.
            visionPortal = builder.build();

            // Set confidence threshold for TFOD recognitions, at any time.
            //tfod.setMinResultConfidence(0.75f);

            // Disable or re-enable the TFOD processor at any time.
            //visionPortal.setProcessorEnabled(tfod, true);

        }   // end method initTfod()

        /**
         * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
         */
        private void telemetryTfod() {

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());
    }

    public void execute(){
        telemetryTfod();

        // Push telemetry to the Driver Station.

        // Save CPU resources; can resume streaming when needed.
        if (gamepad1.dpad_down) {
            visionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
            visionPortal.resumeStreaming();
        }

        // Share the CPU.
        sleep(20);
        Done = true;
    }

    public void end(){

    }

    public boolean isFinished(){
        if (Done){
            return true;
        }
        return false;
    }
}
