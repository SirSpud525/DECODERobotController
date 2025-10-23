package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class Camera {
    public double x, y, z, roll, pitch, yaw;
    public int id;
    public boolean currentlyInSight = false;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;

    public Camera(WebcamName camera) {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(camera)
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public void update() {
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                currentlyInSight = true;
                x = tag.ftcPose.x;
                y = tag.ftcPose.y;
                z = tag.ftcPose.z;
                roll = tag.ftcPose.roll;
                pitch = tag.ftcPose.pitch;
                yaw = tag.ftcPose.yaw;
                id = tag.id;
            } else {
                currentlyInSight = false;
            }
    }

}
