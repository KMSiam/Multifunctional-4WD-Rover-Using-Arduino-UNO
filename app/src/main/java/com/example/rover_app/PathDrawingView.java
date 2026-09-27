package com.example.rover_app;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.PointF;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

import androidx.annotation.Nullable;

import java.util.ArrayList;
import java.util.List;

/**
 * Technical Blueprint Canvas for Autonomous Rover Path Planning.
 * Highly optimized: zero onDraw allocations, pre-compiled grid path,
 * smooth quadratic Bezier splines, adaptive resolution, and space-calibrated timing.
 */
public class PathDrawingView extends View {

    public enum SpaceMode {
        TINY_DESK,      // High precision, micro steps (120-380ms) for small rooms/desks
        STANDARD_FLOOR  // Normal steps (200-900ms) for open floor areas
    }

    public interface PathListener {
        void onPathDrawn(String generatedCommand, int stepCount, long totalDurationMs);
        void onPathCleared();
    }

    private PathListener listener;
    private SpaceMode spaceMode = SpaceMode.TINY_DESK;

    private final Path drawPath = new Path();
    private final Path gridPath = new Path();
    private final List<PointF> rawPoints = new ArrayList<>();
    private final List<Waypoint> waypointMarkers = new ArrayList<>();

    // Pre-allocated Paints
    private final Paint pathPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint pathGlowPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint gridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint startPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint startRingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint endPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint endRingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint waypointTurnPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint waypointStraightPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint waypointRingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint waypointTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint labelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    // Cached dimensional metrics
    private float cachedGridSize;
    private float cachedMarkerRadius;
    private float cachedRingRadius;
    private float cachedLabelOffset;
    private float cachedWaypointRadius;
    private float cachedTurnRadius;
    private float cachedMinMoveDist;
    private float textVerticalOffset;

    // Spline curve smoothing
    private float lastX, lastY;

    public static class Waypoint {
        public final float x;
        public final float y;
        public final int stepIndex;
        public final String label;
        public final boolean isTurn;

        public Waypoint(float x, float y, int stepIndex, String label, boolean isTurn) {
            this.x = x;
            this.y = y;
            this.stepIndex = stepIndex;
            this.label = label;
            this.isTurn = isTurn;
        }
    }

    public PathDrawingView(Context context) {
        super(context);
        init();
    }

    public PathDrawingView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public PathDrawingView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {
        cachedGridSize = dpToPx(28);
        cachedMarkerRadius = dpToPx(11);
        cachedRingRadius = cachedMarkerRadius + dpToPx(3.5f);
        cachedLabelOffset = cachedMarkerRadius + dpToPx(5.5f);
        cachedWaypointRadius = dpToPx(6.5f);
        cachedTurnRadius = dpToPx(8.5f);
        cachedMinMoveDist = dpToPx(4); // Finer sampling during motion

        // Blueprint dashed grid lines
        gridPaint.setColor(Color.parseColor("#E2E8F0"));
        gridPaint.setStyle(Paint.Style.STROKE);
        gridPaint.setStrokeWidth(dpToPx(1f));
        gridPaint.setPathEffect(new DashPathEffect(new float[]{dpToPx(3), dpToPx(4)}, 0));

        // Outer trajectory ambient glow
        pathGlowPaint.setColor(Color.parseColor("#332563EB"));
        pathGlowPaint.setStyle(Paint.Style.STROKE);
        pathGlowPaint.setStrokeWidth(dpToPx(10f));
        pathGlowPaint.setStrokeCap(Paint.Cap.ROUND);
        pathGlowPaint.setStrokeJoin(Paint.Join.ROUND);

        // Core trajectory line
        pathPaint.setColor(Color.parseColor("#2563EB"));
        pathPaint.setStyle(Paint.Style.STROKE);
        pathPaint.setStrokeWidth(dpToPx(4f));
        pathPaint.setStrokeCap(Paint.Cap.ROUND);
        pathPaint.setStrokeJoin(Paint.Join.ROUND);

        // Start marker (Emerald green)
        startPaint.setColor(Color.parseColor("#10B981"));
        startPaint.setStyle(Paint.Style.FILL);

        startRingPaint.setColor(Color.parseColor("#3310B981"));
        startRingPaint.setStyle(Paint.Style.STROKE);
        startRingPaint.setStrokeWidth(dpToPx(4f));

        // End marker (Crimson red)
        endPaint.setColor(Color.parseColor("#EF4444"));
        endPaint.setStyle(Paint.Style.FILL);

        endRingPaint.setColor(Color.parseColor("#33EF4444"));
        endRingPaint.setStyle(Paint.Style.STROKE);
        endRingPaint.setStrokeWidth(dpToPx(4f));

        // Waypoint turn nodes (Amber / Orange)
        waypointTurnPaint.setColor(Color.parseColor("#F59E0B"));
        waypointTurnPaint.setStyle(Paint.Style.FILL);

        // Waypoint straight milestone nodes (Electric Cyan / Sky Blue)
        waypointStraightPaint.setColor(Color.parseColor("#0284C7"));
        waypointStraightPaint.setStyle(Paint.Style.FILL);

        waypointRingPaint.setColor(Color.parseColor("#400284C7"));
        waypointRingPaint.setStyle(Paint.Style.STROKE);
        waypointRingPaint.setStrokeWidth(dpToPx(2.5f));

        waypointTextPaint.setColor(Color.WHITE);
        waypointTextPaint.setTextSize(dpToPx(9f));
        waypointTextPaint.setFakeBoldText(true);
        waypointTextPaint.setTextAlign(Paint.Align.CENTER);
        textVerticalOffset = (waypointTextPaint.descent() + waypointTextPaint.ascent()) / 2f;

        // Start/Finish Labels
        labelPaint.setColor(Color.parseColor("#0F172A"));
        labelPaint.setTextSize(dpToPx(10.5f));
        labelPaint.setFakeBoldText(true);
        labelPaint.setTextAlign(Paint.Align.CENTER);
    }

    public void setPathListener(PathListener listener) {
        this.listener = listener;
    }

    public void setSpaceMode(SpaceMode mode) {
        if (this.spaceMode != mode) {
            this.spaceMode = mode;
            buildWaypointsAndNotify();
            invalidate();
        }
    }

    public SpaceMode getSpaceMode() {
        return spaceMode;
    }

    public void clearPath() {
        rawPoints.clear();
        waypointMarkers.clear();
        drawPath.reset();
        invalidate();
        if (listener != null) {
            listener.onPathCleared();
        }
    }

    public boolean hasPath() {
        return rawPoints.size() >= 2;
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        gridPath.reset();
        for (float x = cachedGridSize; x < w; x += cachedGridSize) {
            gridPath.moveTo(x, 0);
            gridPath.lineTo(x, h);
        }
        for (float y = cachedGridSize; y < h; y += cachedGridSize) {
            gridPath.moveTo(0, y);
            gridPath.lineTo(w, y);
        }
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        // 1. Coordinate grid
        canvas.drawPath(gridPath, gridPaint);

        // 2. Trajectory glow + core path
        canvas.drawPath(drawPath, pathGlowPaint);
        canvas.drawPath(drawPath, pathPaint);

        // 3. Intermediate checkpoint waypoints
        for (int i = 0; i < waypointMarkers.size(); i++) {
            Waypoint wp = waypointMarkers.get(i);
            float radius = wp.isTurn ? cachedTurnRadius : cachedWaypointRadius;
            Paint bgPaint = wp.isTurn ? waypointTurnPaint : waypointStraightPaint;

            canvas.drawCircle(wp.x, wp.y, radius + dpToPx(2f), waypointRingPaint);
            canvas.drawCircle(wp.x, wp.y, radius, bgPaint);
            canvas.drawText(wp.label, wp.x, wp.y - textVerticalOffset, waypointTextPaint);
        }

        // 4. Start and End Markers
        if (!rawPoints.isEmpty()) {
            PointF start = rawPoints.get(0);
            canvas.drawCircle(start.x, start.y, cachedRingRadius, startRingPaint);
            canvas.drawCircle(start.x, start.y, cachedMarkerRadius, startPaint);
            canvas.drawText("START", start.x, start.y - cachedLabelOffset, labelPaint);

            if (rawPoints.size() > 1) {
                PointF end = rawPoints.get(rawPoints.size() - 1);
                canvas.drawCircle(end.x, end.y, cachedRingRadius, endRingPaint);
                canvas.drawCircle(end.x, end.y, cachedMarkerRadius, endPaint);
                canvas.drawText("FINISH", end.x, end.y - cachedLabelOffset, labelPaint);
            }
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        float x = event.getX();
        float y = event.getY();

        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
                rawPoints.clear();
                waypointMarkers.clear();
                drawPath.reset();
                drawPath.moveTo(x, y);
                lastX = x;
                lastY = y;
                rawPoints.add(new PointF(x, y));
                invalidate();
                return true;

            case MotionEvent.ACTION_MOVE:
                if (!rawPoints.isEmpty()) {
                    float dist = (float) Math.hypot(x - lastX, y - lastY);
                    if (dist >= cachedMinMoveDist) {
                        float midX = (x + lastX) / 2;
                        float midY = (y + lastY) / 2;
                        drawPath.quadTo(lastX, lastY, midX, midY);
                        lastX = x;
                        lastY = y;
                        rawPoints.add(new PointF(x, y));
                        invalidate();
                    }
                }
                return true;

            case MotionEvent.ACTION_UP:
                if (!rawPoints.isEmpty()) {
                    drawPath.lineTo(x, y);
                    rawPoints.add(new PointF(x, y));
                    buildWaypointsAndNotify();
                    invalidate();
                }
                return true;
        }

        return super.onTouchEvent(event);
    }

    public void recalculate() {
        buildWaypointsAndNotify();
        invalidate();
    }

    private void buildWaypointsAndNotify() {
        if (rawPoints.size() < 2) return;

        GeneratedPathResult result = processPathCommands();
        if (listener != null && result != null) {
            listener.onPathDrawn(result.commandString, result.stepCount, result.totalDurationMs);
        }
    }

    public String generateRoverCommand() {
        GeneratedPathResult result = processPathCommands();
        return (result != null) ? result.commandString : "S";
    }

    public static class GeneratedPathResult {
        public final String commandString;
        public final int stepCount;
        public final long totalDurationMs;

        public GeneratedPathResult(String commandString, int stepCount, long totalDurationMs) {
            this.commandString = commandString;
            this.stepCount = stepCount;
            this.totalDurationMs = totalDurationMs;
        }
    }

    private GeneratedPathResult processPathCommands() {
        if (rawPoints.size() < 2) {
            return null;
        }

        waypointMarkers.clear();

        // 1. High precision adaptive distance threshold
        // Tiny Desk mode uses 16dp for high waypoint density; Normal uses 26dp
        float stepDist = (spaceMode == SpaceMode.TINY_DESK) ? dpToPx(16f) : dpToPx(26f);
        List<PointF> simplified = new ArrayList<>();
        simplified.add(rawPoints.get(0));

        PointF prev = rawPoints.get(0);
        for (int i = 1; i < rawPoints.size(); i++) {
            PointF cur = rawPoints.get(i);
            float d = (float) Math.hypot(cur.x - prev.x, cur.y - prev.y);
            if (d >= stepDist || i == rawPoints.size() - 1) {
                simplified.add(cur);
                prev = cur;
            }
        }

        if (simplified.size() < 2) {
            simplified.add(rawPoints.get(rawPoints.size() - 1));
        }

        // Limit maximum waypoints to 16 in Tiny mode (10 in Normal) to guarantee buffer safety
        int maxWaypoints = (spaceMode == SpaceMode.TINY_DESK) ? 16 : 10;
        if (simplified.size() > maxWaypoints) {
            List<PointF> reduced = new ArrayList<>();
            reduced.add(simplified.get(0));
            int step = (int) Math.ceil((double) (simplified.size() - 1) / (double) (maxWaypoints - 1));
            for (int i = step; i < simplified.size() - 1; i += step) {
                reduced.add(simplified.get(i));
            }
            reduced.add(simplified.get(simplified.size() - 1));
            simplified = reduced;
        }

        class Step {
            final char dir;
            long duration;
            Step(char dir, long duration) { this.dir = dir; this.duration = duration; }
        }
        List<Step> stepList = new ArrayList<>();

        PointF p0 = simplified.get(0);
        PointF p1 = simplified.get(1);

        double currentHeading = Math.toDegrees(Math.atan2(p1.y - p0.y, p1.x - p0.x));
        float d0 = (float) Math.hypot(p1.x - p0.x, p1.y - p0.y);
        long t0 = calculateForwardTime(d0);
        stepList.add(new Step('F', t0));

        int milestoneCounter = 1;
        // Mark first point
        waypointMarkers.add(new Waypoint(p1.x, p1.y, milestoneCounter++, "1", false));

        double turnThreshold = (spaceMode == SpaceMode.TINY_DESK) ? 14.0 : 18.0;
        long maxMergedForward = (spaceMode == SpaceMode.TINY_DESK) ? 380 : 900;

        for (int i = 1; i < simplified.size() - 1; i++) {
            PointF from = simplified.get(i);
            PointF to = simplified.get(i + 1);

            double targetHeading = Math.toDegrees(Math.atan2(to.y - from.y, to.x - from.x));
            double deltaAngle = targetHeading - currentHeading;

            while (deltaAngle > 180) deltaAngle -= 360;
            while (deltaAngle < -180) deltaAngle += 360;

            // Turn detection: sensitive threshold for tight curves
            if (Math.abs(deltaAngle) >= turnThreshold) {
                long turnMs = calculateTurnTime(deltaAngle);
                char turnDir = (deltaAngle > 0) ? 'R' : 'L';
                stepList.add(new Step(turnDir, turnMs));
                currentHeading = targetHeading;

                waypointMarkers.add(new Waypoint(from.x, from.y, milestoneCounter++, String.valueOf(turnDir), true));
            } else {
                // Straight milestone point
                waypointMarkers.add(new Waypoint(from.x, from.y, milestoneCounter++, String.valueOf(milestoneCounter - 1), false));
            }

            float segmentDist = (float) Math.hypot(to.x - from.x, to.y - from.y);
            if (segmentDist >= dpToPx(6)) {
                long forwardMs = calculateForwardTime(segmentDist);
                if (!stepList.isEmpty() && stepList.get(stepList.size() - 1).dir == 'F') {
                    long merged = stepList.get(stepList.size() - 1).duration + forwardMs;
                    if (merged <= maxMergedForward) {
                        stepList.get(stepList.size() - 1).duration = merged;
                    } else {
                        // Split into separate short forward step to preserve tiny-space safety
                        stepList.add(new Step('F', forwardMs));
                    }
                } else {
                    stepList.add(new Step('F', forwardMs));
                }
            }
        }

        // Build command string
        StringBuilder sb = new StringBuilder();
        long totalDurationMs = 0;

        for (int i = 0; i < stepList.size(); i++) {
            Step s = stepList.get(i);
            if (i > 0) sb.append(",");
            sb.append(s.dir).append(":").append(s.duration);
            totalDurationMs += s.duration + 100; // 100ms settling pause
        }

        if (sb.length() > 0) {
            sb.append(",S");
        } else {
            sb.append("S");
        }

        return new GeneratedPathResult(sb.toString(), stepList.size(), totalDurationMs);
    }

    private long calculateForwardTime(float pixelDist) {
        if (spaceMode == SpaceMode.TINY_DESK) {
            // Short time sequences: 1.2ms/pixel, clamped strictly between 120ms and 380ms
            float msPerPixel = 1.2f;
            long time = Math.round(pixelDist * msPerPixel);
            return Math.max(120, Math.min(380, (time / 10) * 10));
        } else {
            // Standard floor mode: 2.6ms/pixel, clamped between 200ms and 900ms
            float msPerPixel = 2.6f;
            long time = Math.round(pixelDist * msPerPixel);
            return Math.max(200, Math.min(900, (time / 20) * 20));
        }
    }

    private long calculateTurnTime(double deltaAngle) {
        double deg = Math.abs(deltaAngle);
        if (spaceMode == SpaceMode.TINY_DESK) {
            // Calibrated for 12V rover in tiny space: 90 deg = ~230ms
            long ms = Math.round(deg * (230.0 / 90.0));
            return Math.max(90, Math.min(380, (ms / 10) * 10));
        } else {
            // Standard floor mode: 90 deg = ~360ms
            long ms = Math.round(deg * (360.0 / 90.0));
            return Math.max(160, Math.min(750, (ms / 20) * 20));
        }
    }

    private float dpToPx(float dp) {
        return dp * getResources().getDisplayMetrics().density;
    }
}
