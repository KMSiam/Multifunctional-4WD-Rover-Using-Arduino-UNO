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
 * Supports smooth spline drawing, waypoint extraction, turn calculation,
 * and compact command generation for Arduino Uno differential drive.
 */
public class PathDrawingView extends View {

    public interface PathListener {
        void onPathDrawn(String generatedCommand, int stepCount, long totalDurationMs);
        void onPathCleared();
    }

    private PathListener listener;

    private final Path drawPath = new Path();
    private final List<PointF> rawPoints = new ArrayList<>();
    private final List<Waypoint> waypointMarkers = new ArrayList<>();

    // Paints
    private final Paint pathPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint pathGlowPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint gridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint startPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint startRingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint endPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint endRingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint waypointPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint waypointTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint labelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint arrowPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    // Cached dimensional metrics
    private float cachedGridSize;
    private float cachedMarkerRadius;
    private float cachedMinMoveDist;

    // Spline curve smoothing
    private float lastX, lastY;

    public static class Waypoint {
        public final float x;
        public final float y;
        public final int stepIndex;
        public final String label;

        public Waypoint(float x, float y, int stepIndex, String label) {
            this.x = x;
            this.y = y;
            this.stepIndex = stepIndex;
            this.label = label;
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
        cachedMinMoveDist = dpToPx(6);

        // Grid lines
        gridPaint.setColor(Color.parseColor("#E2E8F0"));
        gridPaint.setStyle(Paint.Style.STROKE);
        gridPaint.setStrokeWidth(dpToPx(1f));
        gridPaint.setPathEffect(new DashPathEffect(new float[]{dpToPx(3), dpToPx(4)}, 0));

        // Outer glow
        pathGlowPaint.setColor(Color.parseColor("#332563EB"));
        pathGlowPaint.setStyle(Paint.Style.STROKE);
        pathGlowPaint.setStrokeWidth(dpToPx(10f));
        pathGlowPaint.setStrokeCap(Paint.Cap.ROUND);
        pathGlowPaint.setStrokeJoin(Paint.Join.ROUND);

        // Core trajectory line
        pathPaint.setColor(Color.parseColor("#2563EB"));
        pathPaint.setStyle(Paint.Style.STROKE);
        pathPaint.setStrokeWidth(dpToPx(4.5f));
        pathPaint.setStrokeCap(Paint.Cap.ROUND);
        pathPaint.setStrokeJoin(Paint.Join.ROUND);

        // Start marker (Emerald)
        startPaint.setColor(Color.parseColor("#10B981"));
        startPaint.setStyle(Paint.Style.FILL);

        startRingPaint.setColor(Color.parseColor("#3310B981"));
        startRingPaint.setStyle(Paint.Style.STROKE);
        startRingPaint.setStrokeWidth(dpToPx(4f));

        // End marker (Crimson)
        endPaint.setColor(Color.parseColor("#EF4444"));
        endPaint.setStyle(Paint.Style.FILL);

        endRingPaint.setColor(Color.parseColor("#33EF4444"));
        endRingPaint.setStyle(Paint.Style.STROKE);
        endRingPaint.setStrokeWidth(dpToPx(4f));

        // Waypoint nodes (Amber)
        waypointPaint.setColor(Color.parseColor("#F59E0B"));
        waypointPaint.setStyle(Paint.Style.FILL);

        waypointTextPaint.setColor(Color.WHITE);
        waypointTextPaint.setTextSize(dpToPx(9.5f));
        waypointTextPaint.setFakeBoldText(true);
        waypointTextPaint.setTextAlign(Paint.Align.CENTER);

        // Labels
        labelPaint.setColor(Color.parseColor("#0F172A"));
        labelPaint.setTextSize(dpToPx(10.5f));
        labelPaint.setFakeBoldText(true);
        labelPaint.setTextAlign(Paint.Align.CENTER);

        // Direction chevrons
        arrowPaint.setColor(Color.parseColor("#1D4ED8"));
        arrowPaint.setStyle(Paint.Style.STROKE);
        arrowPaint.setStrokeWidth(dpToPx(2.5f));
        arrowPaint.setStrokeCap(Paint.Cap.ROUND);
        arrowPaint.setStrokeJoin(Paint.Join.ROUND);
    }

    public void setPathListener(PathListener listener) {
        this.listener = listener;
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
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        int width = getWidth();
        int height = getHeight();

        // 1. Draw coordinate grid
        for (float x = cachedGridSize; x < width; x += cachedGridSize) {
            canvas.drawLine(x, 0, x, height, gridPaint);
        }
        for (float y = cachedGridSize; y < height; y += cachedGridSize) {
            canvas.drawLine(0, y, width, y, gridPaint);
        }

        // 2. Draw path glow + core trajectory
        canvas.drawPath(drawPath, pathGlowPaint);
        canvas.drawPath(drawPath, pathPaint);

        // 3. Draw intermediate waypoint markers
        for (Waypoint wp : waypointMarkers) {
            canvas.drawCircle(wp.x, wp.y, dpToPx(7f), waypointPaint);
            float textY = wp.y - ((waypointTextPaint.descent() + waypointTextPaint.ascent()) / 2);
            canvas.drawText(String.valueOf(wp.stepIndex), wp.x, textY, waypointTextPaint);
        }

        // 4. Draw Start and End Markers
        if (!rawPoints.isEmpty()) {
            PointF start = rawPoints.get(0);
            canvas.drawCircle(start.x, start.y, cachedMarkerRadius + dpToPx(3f), startRingPaint);
            canvas.drawCircle(start.x, start.y, cachedMarkerRadius, startPaint);
            canvas.drawText("START", start.x, start.y - cachedMarkerRadius - dpToPx(5f), labelPaint);

            if (rawPoints.size() > 1) {
                PointF end = rawPoints.get(rawPoints.size() - 1);
                canvas.drawCircle(end.x, end.y, cachedMarkerRadius + dpToPx(3f), endRingPaint);
                canvas.drawCircle(end.x, end.y, cachedMarkerRadius, endPaint);
                canvas.drawText("FINISH", end.x, end.y - cachedMarkerRadius - dpToPx(5f), labelPaint);
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

    /**
     * Converts drawn points into simplified waypoints and Arduino sequential commands.
     */
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

    private static class GeneratedPathResult {
        final String commandString;
        final int stepCount;
        final long totalDurationMs;

        GeneratedPathResult(String commandString, int stepCount, long totalDurationMs) {
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

        // 1. Simplify raw points using adaptive distance threshold (40dp)
        float stepDist = dpToPx(40);
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

        // Limit maximum waypoints to 8 to fit within Arduino 64-byte serial buffer
        if (simplified.size() > 8) {
            List<PointF> reduced = new ArrayList<>();
            reduced.add(simplified.get(0));
            int step = (int) Math.ceil((double) (simplified.size() - 1) / 7.0);
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

        int waypointCounter = 1;

        for (int i = 1; i < simplified.size() - 1; i++) {
            PointF from = simplified.get(i);
            PointF to = simplified.get(i + 1);

            double targetHeading = Math.toDegrees(Math.atan2(to.y - from.y, to.x - from.x));
            double deltaAngle = targetHeading - currentHeading;

            while (deltaAngle > 180) deltaAngle -= 360;
            while (deltaAngle < -180) deltaAngle += 360;

            // Turn detection (threshold: 20 degrees)
            if (Math.abs(deltaAngle) >= 20) {
                long turnMs = Math.round(Math.abs(deltaAngle) * (550.0 / 90.0));
                turnMs = Math.max(250, Math.min(1100, (turnMs / 50) * 50));
                char turnDir = (deltaAngle > 0) ? 'R' : 'L';
                stepList.add(new Step(turnDir, turnMs));
                currentHeading = targetHeading;

                waypointMarkers.add(new Waypoint(from.x, from.y, waypointCounter++, String.valueOf(turnDir)));
            }

            float segmentDist = (float) Math.hypot(to.x - from.x, to.y - from.y);
            if (segmentDist >= dpToPx(10)) {
                long forwardMs = calculateForwardTime(segmentDist);
                if (!stepList.isEmpty() && stepList.get(stepList.size() - 1).dir == 'F') {
                    stepList.get(stepList.size() - 1).duration += forwardMs;
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
            totalDurationMs += s.duration + 120; // Includes 120ms settling pause
        }

        if (sb.length() > 0) {
            sb.append(",S");
        } else {
            sb.append("S");
        }

        return new GeneratedPathResult(sb.toString(), stepList.size(), totalDurationMs);
    }

    private long calculateForwardTime(float pixelDist) {
        float msPerPixel = 5.0f;
        long time = Math.round(pixelDist * msPerPixel);
        return Math.max(350, Math.min(2200, (time / 50) * 50));
    }

    private float dpToPx(float dp) {
        return dp * getResources().getDisplayMetrics().density;
    }
}
