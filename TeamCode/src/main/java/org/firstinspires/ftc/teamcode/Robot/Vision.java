package org.firstinspires.ftc.teamcode.Robot;
/*
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class Vision {
    HardwareMap hwMap;
    WebcamName colorCam;
    WebcamName tagCam;
    Thread ColorLocator = new Thread(this::fetchColorBlobs);
    Thread TagLocator = new Thread(this::fetchTagDetections);
    private ColorBlobLocatorProcessor pollenDetector;
    private ColorBlobLocatorProcessor redNectarDetector;
    private ColorBlobLocatorProcessor blueNectarDetector;
    private AprilTagProcessor tagProcessor;
    private VisionPortal colorPortal;
    private VisionPortal tagPortal;


    public BlockingQueue<List<ColorBlobLocatorProcessor.Blob>> pollenQueue = new LinkedBlockingQueue<>();
    public BlockingQueue<List<ColorBlobLocatorProcessor.Blob>> redNectarQueue = new LinkedBlockingQueue<>();
    public BlockingQueue<List<ColorBlobLocatorProcessor.Blob>> blueNectarQueue = new LinkedBlockingQueue<>();
    public BlockingQueue<List<AprilTagDetection>> tagQueue = new LinkedBlockingQueue<>();

    public Vision(HardwareMap hardwareMap){
        hwMap = hardwareMap;
        colorCam = hwMap.get(WebcamName.class, "colorCam");
        tagCam = hwMap.get(WebcamName.class, "tagCam");
    }

    public void startDetectors(){
        pollenDetector = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();
        redNectarDetector = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        blueNectarDetector = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .build();


        colorPortal = new VisionPortal.Builder()
                .addProcessors(pollenDetector, redNectarDetector, blueNectarDetector)
                .setCamera(colorCam)
                .build();

        tagPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(tagCam)
                .build();


        ColorLocator.start();
        TagLocator.start();
    }

    public void stopDetectors(){
        ColorLocator.interrupt();
        TagLocator.interrupt();

        if (colorPortal != null) colorPortal.close();
        if (tagPortal != null) tagPortal.close();
    }

    public boolean isScorableCell(){
        // This function assumes that we're already in proper position so it just checks
        // for tags, and any visible tag must be ours
        return !tagProcessor.getDetections().isEmpty();
    }

    public ColorBlobLocatorProcessor.Blob getLargestBlob(BlockingQueue<List<ColorBlobLocatorProcessor.Blob>> queue) throws InterruptedException{
        List<ColorBlobLocatorProcessor.Blob> blobs = queue.take();
        if (blobs.isEmpty()) return null;
        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);
        return blobs.get(0);
    }

    public android.graphics.Point getBlobCenter (ColorBlobLocatorProcessor.Blob blob){
        return new android.graphics.Point((int) blob.getBoxFit().center.x, (int) blob.getBoxFit().center.y);
    }
    public double getBlobSize(ColorBlobLocatorProcessor.Blob blob){
        return blob.getBoxFit().size.area();
    }

    //================================================================================================================================================
    
    private void fetchColorBlobs() {
        while(!Thread.currentThread().isInterrupted()){
            try {
                pollenQueue.put(pollenDetector.getBlobs());
                redNectarQueue.put(redNectarDetector.getBlobs());
                blueNectarQueue.put(blueNectarDetector.getBlobs());
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    private void fetchTagDetections(){
        while(!Thread.currentThread().isInterrupted()){
            try {
                tagQueue.put(tagProcessor.getDetections());
            } catch (InterruptedException e){
                Thread.currentThread().interrupt();
            }
        }
    }

}*/
