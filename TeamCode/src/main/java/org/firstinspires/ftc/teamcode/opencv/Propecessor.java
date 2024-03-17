package org.firstinspires.ftc.teamcode.opencv;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class Propecessor implements VisionProcessor {

    private final Telemetry telemetry;
    Range leftXRange;
    int leftHeight;
    Range centerXRange;
    int centerHeight;
    public int horizontalTolerance = 30;
    public boolean robotAlignLeft = false;
    public boolean isRed = true;
    public double detectThreshold = 50;
    public double unsavoryChannelFactor = 0.5;

    public enum Spike {
        LEFT, CENTER, RIGHT;

        public Spike mirror() {
            switch (this) {
                case LEFT:
                    return RIGHT;
                case CENTER:
                    return CENTER;
                case RIGHT:
                    return LEFT;
            }
            throw new IllegalStateException("poop");
        }
    }

    public Spike selectedSpike;
    private Rect spikeLocLeft;
    private Rect spikeLocCenter;

    public Propecessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        if (robotAlignLeft) {
            leftXRange = new Range(170, 285);
            leftHeight = 120;
            centerXRange = new Range(490, 595);
            centerHeight = 110;
        } else {
            leftXRange = new Range(0, 115);
            leftHeight = 120;
            centerXRange = new Range(390, 495);
            centerHeight = 110;
        }
    }

    class Tuple<A, B> {
        A first;
        B second;

        public Tuple(A first, B second) {
            this.first = first;
            this.second = second;
        }
    }

    private Tuple<Double, Point> processSide(Mat subframe, int sectionWidth, int sectionHeight) {
        List<Mat> channels = new ArrayList<>();
        Core.split(subframe, channels);
        // put coi at first position
        if (!isRed) {
            Mat red = channels.get(0);
            channels.set(0, channels.get(2));
            channels.set(2, red);
        }
        // roi so the kernels don't go out of bounds
        Rect roi = new Rect(0, 0, subframe.width() - sectionWidth + 1, subframe.height() - sectionHeight + 1);
        Size size = new Size(sectionWidth, sectionHeight);
        for (int i = 0; i < channels.size(); i++) {
            double scale = i == 0 ? 1 : unsavoryChannelFactor;
            channels.get(i).convertTo(channels.get(i), CvType.CV_64F, scale);
            Imgproc.boxFilter(new Mat(channels.get(i), roi), channels.get(i), -1, size, new Point(0, 0), true);
        }
        Core.subtract(channels.get(0), channels.get(1), channels.get(0));
        Core.subtract(channels.get(0), channels.get(2), channels.get(0));
        Core.MinMaxLocResult res = Core.minMaxLoc(channels.get(0));
        return new Tuple<>(res.maxVal, res.maxLoc);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Tuple<Double, Point> leftRes = processSide(frame.submat(
                        new Rect(Math.max(0, leftXRange.start - horizontalTolerance), 0,
                                leftXRange.size() + 2 * horizontalTolerance, frame.height())).clone(),
                leftXRange.size(), leftHeight);
        double avgLeft = leftRes.first;
        spikeLocLeft = new Rect(
                (int) leftRes.second.x + leftXRange.start - horizontalTolerance, (int) leftRes.second.y,
                leftXRange.size(), leftHeight);
        Tuple<Double, Point> centerRes = processSide(frame.submat(
                        new Rect(Math.max(0, centerXRange.start - horizontalTolerance), 0,
                                centerXRange.size() + 2 * horizontalTolerance, frame.height())).clone(),
                centerXRange.size(), centerHeight);
        double avgCenter = centerRes.first;
        spikeLocCenter = new Rect(
                (int) centerRes.second.x + centerXRange.start - horizontalTolerance, (int) centerRes.second.y,
                centerXRange.size(), centerHeight);
        // telemetry.addData("left", avgLeft);
        // telemetry.addData("center", avgCenter);
        Spike select;
        if (avgLeft > detectThreshold) {
            if (avgCenter > detectThreshold) {
                select = Spike.CENTER;
            } else {
                select = Spike.LEFT;
            }
        } else {
            if (avgCenter > detectThreshold) {
                select = Spike.CENTER;
            } else {
                select = Spike.RIGHT;
            }
        }
        return select;
    }

    private Rect scaleRect(Rect r, float f) {
        return new Rect((int) (r.x * f), (int) (r.y * f), (int) (r.width * f), (int) (r.height * f));
    }

    private android.graphics.Rect convertRects(Rect rect) {
        return new android.graphics.Rect(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity, Object userContext) {
        selectedSpike = (Spike) userContext;
        telemetry.addData("spike", selectedSpike);
        Paint paint = new Paint();
        paint.setColor(Color.GREEN);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(5);
        canvas.drawRect(convertRects(scaleRect(new Rect(leftXRange.start - horizontalTolerance, 0,
                leftXRange.size() + 2 * horizontalTolerance, 480), scaleBmpPxToCanvasPx)), paint);
        canvas.drawRect(convertRects(scaleRect(new Rect(centerXRange.start - horizontalTolerance, 0,
                centerXRange.size() + 2 * horizontalTolerance, 480), scaleBmpPxToCanvasPx)), paint);
        paint.setColor(Color.RED);
        canvas.drawRect(convertRects(scaleRect(spikeLocLeft, scaleBmpPxToCanvasPx)), paint);
        canvas.drawRect(convertRects(scaleRect(spikeLocCenter, scaleBmpPxToCanvasPx)), paint);
    }
}
