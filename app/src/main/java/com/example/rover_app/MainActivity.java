package com.example.rover_app;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.graphics.drawable.GradientDrawable;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.Spannable;
import android.text.SpannableString;
import android.text.TextUtils;
import android.text.style.ForegroundColorSpan;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.content.ContextCompat;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

import com.google.android.material.button.MaterialButton;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.Method;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.UUID;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Arduino Uno Rover Controller with 3 Operating Modes:
 * 1. MANUAL CONTROL MODE (D-Pad + 500ms auto-stop pulses + Speed regulation slider)
 * 2. OBSTACLE AVOIDANCE MODE (Autonomous HC-SR04 & SG90 servo telemetry monitor)
 * 3. DRAW-A-PATH MODE (Touch drawing canvas generating sequential differential-drive commands)
 *
 * Built on top of the verified Bluetooth Classic SPP/RFCOMM communication engine.
 */
public class MainActivity extends AppCompatActivity {

    // Standard Bluetooth Classic SPP UUID
    private static final UUID BLUETOOTH_SPP = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    // Standard BLE Serial Service UUIDs (fallback support)
    private static final UUID BLE_CCCD           = UUID.fromString("00002902-0000-1000-8000-00805F9B34FB");
    private static final UUID BLE_CC254X_SERVICE = UUID.fromString("0000FFE0-0000-1000-8000-00805F9B34FB");
    private static final UUID BLE_CC254X_CHAR    = UUID.fromString("0000FFE1-0000-1000-8000-00805F9B34FB");
    private static final UUID BLE_NRF_SERVICE    = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    private static final UUID BLE_NRF_CHAR_WRITE = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
    private static final UUID BLE_NRF_CHAR_READ  = UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

    // Pre-compiled telemetry distance detection pattern
    private static final Pattern TELEMETRY_DIST_PATTERN =
            Pattern.compile("(?i)(?:dist(?:ance)?|d)?[\\s:=]*(\\d{1,3})\\s*(?:cm)?");

    // Rolling terminal buffer caps to prevent memory bloat during prolonged telemetry streams
    private static final int MAX_TERMINAL_CHARS = 20000;
    private static final int TRIM_TARGET_CHARS = 14000;

    private static final int COLOR_RX = Color.parseColor("#4ADE80");
    private static final int COLOR_TX = Color.parseColor("#FBBF24");
    private static final int COLOR_STATUS = Color.parseColor("#38BDF8");
    private static final int COLOR_ERROR = Color.parseColor("#F87171");

    public enum RoverMode {
        MANUAL,
        OBSTACLE,
        PATH
    }
    private RoverMode currentMode = RoverMode.MANUAL;

    private Spinner spinnerPairedDevices;
    private MaterialButton btnRefresh;
    private TextView tvConnectionBadge;
    private MaterialButton btnClearLog;
    private MaterialButton btnConnectToggle;
    private View bannerDisconnected;

    private MaterialButton btnTabManual;
    private MaterialButton btnTabObstacle;
    private MaterialButton btnTabPath;

    private View panelManual;
    private View panelObstacle;
    private View panelPath;

    private TextView tvSpeedValue;
    private SeekBar sbSpeed;
    private MaterialButton btnSpeed80, btnSpeed120, btnSpeed180, btnSpeed255;
    private MaterialButton btnManualFwd, btnManualBack, btnManualLeft, btnManualRight, btnManualStop;

    private TextView tvObstacleBadge;
    private TextView tvObstacleStatus;
    private TextView tvObstacleSubtext;
    private ScrollView scrollObstacleTerminal;
    private TextView tvObstacleTerminal;
    private MaterialButton btnClearObstacleLog;
    private MaterialButton btnObstacleStart;
    private MaterialButton btnObstacleStop;
    private final StringBuilder rxLineBuffer = new StringBuilder();

    private PathDrawingView pathDrawingView;
    private TextView tvPathPreview;
    private TextView tvPathMetrics;
    private TextView tvPathStatusBadge;
    private MaterialButton btnClearPath, btnGeneratePath, btnSendPath, btnPathStop;

    private View btnToggleConsole;
    private TextView tvConsoleHeader;
    private TextView tvConsoleStatus;
    private View layoutConsoleContent;
    private ScrollView scrollTerminal;
    private TextView tvTerminal;
    private MaterialButton btnNewlineToggle;
    private EditText etSendCommand;
    private MaterialButton btnSend;
    private boolean isConsoleExpanded = false;

    private BluetoothAdapter bluetoothAdapter;
    private final List<BluetoothDevice> pairedDevices = new ArrayList<>();
    private final List<String> pairedDisplayNames = new ArrayList<>();
    private ArrayAdapter<String> deviceSpinnerAdapter;

    private BluetoothSocket classicSocket;
    private InputStream classicInputStream;
    private OutputStream classicOutputStream;

    private BluetoothGatt bleGatt;
    private BluetoothGattCharacteristic bleWriteChar;
    private BluetoothGattCharacteristic bleReadChar;

    private final ExecutorService executor = Executors.newCachedThreadPool();
    private final ExecutorService sendExecutor = Executors.newSingleThreadExecutor();
    private final Object socketLock = new Object();
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private volatile boolean connected = false;
    private volatile boolean pendingConnect = false;
    private volatile boolean userRequestedDisconnect = false;
    private BluetoothDevice lastConnectedDevice = null;
    private int reconnectAttempts = 0;
    private static final int MAX_RECONNECT_ATTEMPTS = 3;

    private final Runnable autoReconnectRunnable = new Runnable() {
        @Override
        public void run() {
            if (!connected && !userRequestedDisconnect && lastConnectedDevice != null && reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
                reconnectAttempts++;
                String devName = lastConnectedDevice.getName() != null ? lastConnectedDevice.getName() : lastConnectedDevice.getAddress();
                appendStatus("⚡ Auto-reconnecting to " + devName + " (Attempt " + reconnectAttempts + "/" + MAX_RECONNECT_ATTEMPTS + ")...");
                connect(lastConnectedDevice);
            }
        }
    };

    private final SimpleDateFormat timeFormat = new SimpleDateFormat("HH:mm:ss", Locale.getDefault());

    private enum NewlineMode {
        LF("LF (\\n)", "\n"),
        CRLF("CRLF (\\r\\n)", "\r\n"),
        NONE("NONE", "");

        final String label;
        final String value;
        NewlineMode(String label, String value) {
            this.label = label;
            this.value = value;
        }
    }
    private NewlineMode currentNewlineMode = NewlineMode.LF;

    private final BroadcastReceiver aclDisconnectReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (BluetoothDevice.ACTION_ACL_DISCONNECTED.equals(intent.getAction())) {
                if (connected) {
                    appendStatus("Hardware disconnected (ACL disconnected).");
                    disconnect(false);
                }
            }
        }
    };

    private final ActivityResultLauncher<String[]> permissionLauncher =
            registerForActivityResult(new ActivityResultContracts.RequestMultiplePermissions(), result -> {
                boolean allGranted = true;
                for (Boolean granted : result.values()) {
                    if (!granted) {
                        allGranted = false;
                        break;
                    }
                }
                if (allGranted) {
                    appendStatus("Bluetooth permissions granted.");
                    initBluetooth();
                } else {
                    appendError("Bluetooth permissions denied. Cannot connect to HC-05.");
                    Toast.makeText(this, "Bluetooth permissions required", Toast.LENGTH_LONG).show();
                }
            });

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Apply system window insets so top and bottom navigation bars don't overlap system bars or keyboard
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.root_layout), (v, windowInsets) -> {
            Insets sysBars = windowInsets.getInsets(WindowInsetsCompat.Type.systemBars());
            Insets ime = windowInsets.getInsets(WindowInsetsCompat.Type.ime());
            View topBar = findViewById(R.id.top_navigation_bar);
            if (topBar != null) {
                topBar.setPadding(
                        topBar.getPaddingLeft(),
                        sysBars.top + dpToPx(6),
                        topBar.getPaddingRight(),
                        topBar.getPaddingBottom()
                );
            }
            View bottomBar = findViewById(R.id.bottom_navigation_bar);
            if (bottomBar != null) {
                int bottomPadding = (ime.bottom > 0) ? (ime.bottom + dpToPx(6)) : (sysBars.bottom + dpToPx(8));
                bottomBar.setPadding(
                        bottomBar.getPaddingLeft(),
                        bottomBar.getPaddingTop(),
                        bottomBar.getPaddingRight(),
                        bottomPadding
                );
            }
            return windowInsets;
        });

        initViews();
        setupListeners();
        registerDisconnectReceiver();
        checkPermissionsAndStart();
    }

    private void initViews() {
        spinnerPairedDevices = findViewById(R.id.spinner_paired_devices);
        btnRefresh = findViewById(R.id.btn_refresh);
        tvConnectionBadge = findViewById(R.id.tv_connection_badge);
        btnClearLog = findViewById(R.id.btn_clear_log);
        btnConnectToggle = findViewById(R.id.btn_connect_toggle);
        bannerDisconnected = findViewById(R.id.banner_disconnected);

        btnTabManual = findViewById(R.id.btn_tab_manual);
        btnTabObstacle = findViewById(R.id.btn_tab_obstacle);
        btnTabPath = findViewById(R.id.btn_tab_path);

        panelManual = findViewById(R.id.panel_manual);
        panelObstacle = findViewById(R.id.panel_obstacle);
        panelPath = findViewById(R.id.panel_path);

        tvSpeedValue = findViewById(R.id.tv_speed_value);
        sbSpeed = findViewById(R.id.sb_speed);
        btnSpeed80 = findViewById(R.id.btn_speed_80);
        btnSpeed120 = findViewById(R.id.btn_speed_120);
        btnSpeed180 = findViewById(R.id.btn_speed_180);
        btnSpeed255 = findViewById(R.id.btn_speed_255);
        btnManualFwd = findViewById(R.id.btn_manual_fwd);
        btnManualBack = findViewById(R.id.btn_manual_back);
        btnManualLeft = findViewById(R.id.btn_manual_left);
        btnManualRight = findViewById(R.id.btn_manual_right);
        btnManualStop = findViewById(R.id.btn_manual_stop);

        tvObstacleBadge = findViewById(R.id.tv_obstacle_badge);
        tvObstacleStatus = findViewById(R.id.tv_obstacle_status);
        tvObstacleSubtext = findViewById(R.id.tv_obstacle_subtext);
        scrollObstacleTerminal = findViewById(R.id.scroll_obstacle_terminal);
        tvObstacleTerminal = findViewById(R.id.tv_obstacle_terminal);
        btnClearObstacleLog = findViewById(R.id.btn_clear_obstacle_log);
        btnObstacleStart = findViewById(R.id.btn_obstacle_start);
        btnObstacleStop = findViewById(R.id.btn_obstacle_stop);

        if (btnClearObstacleLog != null) {
            btnClearObstacleLog.setOnClickListener(v -> {
                if (tvObstacleTerminal != null) {
                    tvObstacleTerminal.setText("");
                }
            });
        }

        pathDrawingView = findViewById(R.id.path_drawing_view);
        tvPathPreview = findViewById(R.id.tv_path_preview);
        tvPathMetrics = findViewById(R.id.tv_path_metrics);
        tvPathStatusBadge = findViewById(R.id.tv_path_status_badge);
        btnClearPath = findViewById(R.id.btn_clear_path);
        btnGeneratePath = findViewById(R.id.btn_generate_path);
        btnSendPath = findViewById(R.id.btn_send_path);
        btnPathStop = findViewById(R.id.btn_path_stop);

        btnToggleConsole = findViewById(R.id.btn_toggle_console);
        tvConsoleHeader = findViewById(R.id.tv_console_header);
        tvConsoleStatus = findViewById(R.id.tv_console_status);
        layoutConsoleContent = findViewById(R.id.layout_console_content);
        scrollTerminal = findViewById(R.id.scroll_terminal);
        tvTerminal = findViewById(R.id.tv_terminal);
        btnNewlineToggle = findViewById(R.id.btn_newline_toggle);
        etSendCommand = findViewById(R.id.et_send_command);
        btnSend = findViewById(R.id.btn_send);

        deviceSpinnerAdapter = new ArrayAdapter<>(this, R.layout.item_spinner, pairedDisplayNames);
        deviceSpinnerAdapter.setDropDownViewResource(R.layout.item_spinner_dropdown);
        spinnerPairedDevices.setAdapter(deviceSpinnerAdapter);

        btnNewlineToggle.setText(currentNewlineMode.label);
        btnNewlineToggle.setOnClickListener(v -> {
            if (currentNewlineMode == NewlineMode.LF) {
                currentNewlineMode = NewlineMode.CRLF;
            } else if (currentNewlineMode == NewlineMode.CRLF) {
                currentNewlineMode = NewlineMode.NONE;
            } else {
                currentNewlineMode = NewlineMode.LF;
            }
            btnNewlineToggle.setText(currentNewlineMode.label);
            Toast.makeText(this, "Console Newline: " + currentNewlineMode.label, Toast.LENGTH_SHORT).show();
        });

        updateUiState(ConnectionState.DISCONNECTED, null);
        updateModeUi(RoverMode.MANUAL);
        appendStatus("Rover Controller ready. Select HC-05 and Connect.");
    }

    private void setupListeners() {
        btnRefresh.setOnClickListener(v -> refreshPairedDevices());

        btnConnectToggle.setOnClickListener(v -> {
            if (connected || pendingConnect) {
                userRequestedDisconnect = true;
                mainHandler.removeCallbacks(autoReconnectRunnable);
                disconnect(true);
            } else {
                userRequestedDisconnect = false;
                reconnectAttempts = 0;
                int position = spinnerPairedDevices.getSelectedItemPosition();
                if (position >= 0 && position < pairedDevices.size()) {
                    BluetoothDevice device = pairedDevices.get(position);
                    connect(device);
                } else {
                    Toast.makeText(this, "Select a paired device first", Toast.LENGTH_SHORT).show();
                }
            }
        });

        btnClearLog.setOnClickListener(v -> tvTerminal.setText(""));

        btnTabManual.setOnClickListener(v -> switchMode(RoverMode.MANUAL));
        btnTabObstacle.setOnClickListener(v -> switchMode(RoverMode.OBSTACLE));
        btnTabPath.setOnClickListener(v -> switchMode(RoverMode.PATH));

        btnManualFwd.setOnClickListener(v -> sendRoverCommand("F"));
        btnManualBack.setOnClickListener(v -> sendRoverCommand("B"));
        btnManualLeft.setOnClickListener(v -> sendRoverCommand("L"));
        btnManualRight.setOnClickListener(v -> sendRoverCommand("R"));
        btnManualStop.setOnClickListener(v -> triggerEmergencyStop());

        sbSpeed.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                tvSpeedValue.setText(progress + " / 255");
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {}

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                setSpeed(seekBar.getProgress());
            }
        });

        btnSpeed80.setOnClickListener(v -> setSpeed(80));
        btnSpeed120.setOnClickListener(v -> setSpeed(120));
        btnSpeed180.setOnClickListener(v -> setSpeed(180));
        btnSpeed255.setOnClickListener(v -> setSpeed(255));

        if (btnObstacleStart != null) {
            btnObstacleStart.setOnClickListener(v -> {
                sendRoverCommand("O");
                if (tvObstacleStatus != null) {
                    tvObstacleStatus.setText("● Autonomous Mode Active (Scanning & Driving)");
                    tvObstacleStatus.setTextColor(ContextCompat.getColor(this, R.color.status_connected));
                }
                if (tvObstacleSubtext != null) {
                    tvObstacleSubtext.setText("Command 'O' Sent • Monitoring ultrasonic sensors & telemetry...");
                }
                if (tvObstacleBadge != null) {
                    tvObstacleBadge.setText("ACTIVE");
                }
                Toast.makeText(this, "▶ Obstacle Avoidance Started!", Toast.LENGTH_SHORT).show();
            });
        }

        btnObstacleStop.setOnClickListener(v -> triggerEmergencyStop());

        pathDrawingView.setPathListener(new PathDrawingView.PathListener() {
            @Override
            public void onPathDrawn(String generatedCommand, int stepCount, long totalDurationMs) {
                tvPathPreview.setText(generatedCommand);
                if (tvPathMetrics != null) {
                    tvPathMetrics.setText(stepCount + " Steps • Est. " + String.format(Locale.US, "%.1fs", totalDurationMs / 1000.0f));
                }
                if (tvPathStatusBadge != null) {
                    tvPathStatusBadge.setText("READY");
                    tvPathStatusBadge.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.status_connected));
                }
            }

            @Override
            public void onPathCleared() {
                tvPathPreview.setText("Draw on canvas above to generate path...");
                if (tvPathMetrics != null) {
                    tvPathMetrics.setText("0 Steps • Est. 0.0s");
                }
                if (tvPathStatusBadge != null) {
                    tvPathStatusBadge.setText("STANDBY");
                    tvPathStatusBadge.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.text_secondary));
                }
            }
        });

        btnClearPath.setOnClickListener(v -> pathDrawingView.clearPath());

        btnGeneratePath.setOnClickListener(v -> {
            if (pathDrawingView.hasPath()) {
                String cmd = pathDrawingView.generateRoverCommand();
                tvPathPreview.setText(cmd);
                if (tvPathStatusBadge != null) {
                    tvPathStatusBadge.setText("READY");
                    tvPathStatusBadge.setTextColor(ContextCompat.getColor(this, R.color.status_connected));
                }
                Toast.makeText(this, "Path generated!", Toast.LENGTH_SHORT).show();
            } else {
                Toast.makeText(this, "Draw a path on the canvas first", Toast.LENGTH_SHORT).show();
            }
        });

        btnSendPath.setOnClickListener(v -> {
            String pathCmd = tvPathPreview.getText().toString().trim();
            if (pathCmd.startsWith("F:") && pathCmd.contains("S")) {
                if (tvPathStatusBadge != null) {
                    tvPathStatusBadge.setText("TRANSMITTING");
                    tvPathStatusBadge.setTextColor(ContextCompat.getColor(this, R.color.status_connecting));
                }
                sendRoverCommand("P");

                mainHandler.postDelayed(() -> {
                    sendRoverCommand(pathCmd);
                    if (tvPathStatusBadge != null) {
                        tvPathStatusBadge.setText("EXECUTING");
                        tvPathStatusBadge.setTextColor(ContextCompat.getColor(this, R.color.primary));
                    }
                    Toast.makeText(this, "🚀 Path transmitted to rover!", Toast.LENGTH_SHORT).show();
                }, 150);
            } else {
                Toast.makeText(this, "Draw a valid path on canvas first", Toast.LENGTH_SHORT).show();
            }
        });

        btnPathStop.setOnClickListener(v -> triggerEmergencyStop());

        btnToggleConsole.setOnClickListener(v -> toggleConsole());

        btnSend.setOnClickListener(v -> {
            String text = etSendCommand.getText().toString();
            if (!TextUtils.isEmpty(text)) {
                sendRoverCommand(text);
                etSendCommand.setText("");
            }
        });

        etSendCommand.setOnEditorActionListener((v, actionId, event) -> {
            if (actionId == EditorInfo.IME_ACTION_SEND) {
                btnSend.performClick();
                return true;
            }
            return false;
        });
    }

    /**
     * Emergency Stop across all modes:
     * - Sends redundant S burst over Bluetooth to immediately halt motors
     * - Updates UI status banner to STOPPED
     */
    private void triggerEmergencyStop() {
        sendRoverCommand("S", true);
        if (tvObstacleStatus != null && currentMode == RoverMode.OBSTACLE) {
            tvObstacleStatus.setText("■ EMERGENCY STOP (Braked)");
            tvObstacleStatus.setTextColor(ContextCompat.getColor(this, R.color.danger));
        }
        if (tvObstacleSubtext != null && currentMode == RoverMode.OBSTACLE) {
            tvObstacleSubtext.setText("Motors halted immediately • Tap 'OBSTACLE' tab to resume");
        }
        if (tvObstacleBadge != null && currentMode == RoverMode.OBSTACLE) {
            tvObstacleBadge.setText("STOPPED");
        }
        if (tvPathStatusBadge != null && currentMode == RoverMode.PATH) {
            tvPathStatusBadge.setText("HALTED");
            tvPathStatusBadge.setTextColor(ContextCompat.getColor(this, R.color.danger));
        }
        Toast.makeText(this, "■ EMERGENCY STOP ACTIVATED", Toast.LENGTH_SHORT).show();
    }

    /**
     * Switch Rover Mode safely:
     * 1. If tapping the current mode tab, re-transmits mode command (robust recovery).
     * 2. If switching modes: sends S\n burst, waits 220ms for settling, then sends target mode.
     */
    private void switchMode(RoverMode targetMode) {
        boolean wasSameMode = (targetMode == currentMode);
        currentMode = targetMode;
        updateModeUi(targetMode);

        if (wasSameMode) {
            switch (targetMode) {
                case MANUAL:
                    sendRoverCommand("M");
                    Toast.makeText(this, "Re-sent MANUAL mode (M)", Toast.LENGTH_SHORT).show();
                    break;
                case OBSTACLE:
                    Toast.makeText(this, "Rover in safe Standby. Tap 'START MODE (O)' to begin.", Toast.LENGTH_SHORT).show();
                    break;
                case PATH:
                    sendRoverCommand("P");
                    Toast.makeText(this, "Re-sent PATH mode (P)", Toast.LENGTH_SHORT).show();
                    break;
            }
            return;
        }

        sendRoverCommand("S", true);

        mainHandler.postDelayed(() -> {
            switch (targetMode) {
                case MANUAL:
                    sendRoverCommand("M");
                    break;
                case OBSTACLE:
                    if (tvObstacleStatus != null) {
                        tvObstacleStatus.setText("○ Standby — Tap 'START' to Begin");
                        tvObstacleStatus.setTextColor(ContextCompat.getColor(this, R.color.accent));
                    }
                    if (tvObstacleSubtext != null) {
                        tvObstacleSubtext.setText("Rover in safe standby • Tap green START button below to engage");
                    }
                    if (tvObstacleBadge != null) {
                        tvObstacleBadge.setText("STANDBY");
                    }
                    Toast.makeText(this, "Obstacle Screen (Standby). Tap START to begin.", Toast.LENGTH_SHORT).show();
                    break;
                case PATH:
                    sendRoverCommand("P");
                    break;
            }
        }, 220);
    }

    private void updateModeUi(RoverMode mode) {
        int primaryColor = ContextCompat.getColor(this, R.color.primary);
        int cardBgColor = ContextCompat.getColor(this, R.color.card_bg);
        int whiteColor = ContextCompat.getColor(this, R.color.white);
        int textSecColor = ContextCompat.getColor(this, R.color.text_secondary);

        btnTabManual.setBackgroundTintList(ColorStateList.valueOf(mode == RoverMode.MANUAL ? primaryColor : cardBgColor));
        btnTabManual.setTextColor(mode == RoverMode.MANUAL ? whiteColor : textSecColor);
        btnTabManual.setIconTint(ColorStateList.valueOf(mode == RoverMode.MANUAL ? whiteColor : textSecColor));

        btnTabObstacle.setBackgroundTintList(ColorStateList.valueOf(mode == RoverMode.OBSTACLE ? primaryColor : cardBgColor));
        btnTabObstacle.setTextColor(mode == RoverMode.OBSTACLE ? whiteColor : textSecColor);
        btnTabObstacle.setIconTint(ColorStateList.valueOf(mode == RoverMode.OBSTACLE ? whiteColor : textSecColor));

        btnTabPath.setBackgroundTintList(ColorStateList.valueOf(mode == RoverMode.PATH ? primaryColor : cardBgColor));
        btnTabPath.setTextColor(mode == RoverMode.PATH ? whiteColor : textSecColor);
        btnTabPath.setIconTint(ColorStateList.valueOf(mode == RoverMode.PATH ? whiteColor : textSecColor));

        panelManual.setVisibility(mode == RoverMode.MANUAL ? View.VISIBLE : View.GONE);
        panelObstacle.setVisibility(mode == RoverMode.OBSTACLE ? View.VISIBLE : View.GONE);
        panelPath.setVisibility(mode == RoverMode.PATH ? View.VISIBLE : View.GONE);
    }

    private void setSpeed(int speed) {
        speed = Math.max(0, Math.min(255, speed));
        sbSpeed.setProgress(speed);
        tvSpeedValue.setText(speed + " / 255");
        sendRoverCommand("V:" + speed);
    }

    private void toggleConsole() {
        isConsoleExpanded = !isConsoleExpanded;
        layoutConsoleContent.setVisibility(isConsoleExpanded ? View.VISIBLE : View.GONE);
        tvConsoleHeader.setText(isConsoleExpanded ? "▼ Arduino Live Serial Monitor" : "▶ Arduino Live Serial Monitor");
        tvConsoleStatus.setText(isConsoleExpanded ? "Tap to collapse" : "Tap to expand");
        if (isConsoleExpanded) {
            scrollToBottom();
        }
    }

    /**
     * Send command to Arduino Rover.
     * All commands are automatically terminated with \n.
     */
    @SuppressLint("MissingPermission")
    private void sendRoverCommand(String cmd) {
        sendRoverCommand(cmd, false);
    }

    @SuppressLint("MissingPermission")
    private void sendRoverCommand(String cmd, boolean isUrgentStop) {
        if (!connected) {
            Toast.makeText(this, "Rover is not connected", Toast.LENGTH_SHORT).show();
            return;
        }

        final String payload = cmd.endsWith("\n") ? cmd : (cmd + "\n");
        final byte[] data = payload.getBytes(StandardCharsets.UTF_8);

        if (bleGatt != null && bleWriteChar != null) {
            bleWriteChar.setValue(data);
            bleGatt.writeCharacteristic(bleWriteChar);
            appendTx(cmd.trim());
            return;
        }

        sendExecutor.execute(() -> {
            synchronized (socketLock) {
                if (classicOutputStream != null && connected) {
                    try {
                        classicOutputStream.write(data);
                        classicOutputStream.flush();
                        if (isUrgentStop) {
                            try { Thread.sleep(25); } catch (InterruptedException ignored) {}
                            classicOutputStream.write("S\n".getBytes(StandardCharsets.UTF_8));
                            classicOutputStream.flush();
                        }
                        mainHandler.post(() -> appendTx(cmd.trim()));
                    } catch (IOException e) {
                        mainHandler.post(() -> appendError("Send failed: " + e.getMessage()));
                    }
                }
            }
        });
    }

    private void checkPermissionsAndStart() {
        List<String> requiredPermissions = new ArrayList<>();
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT)
                    != PackageManager.PERMISSION_GRANTED) {
                requiredPermissions.add(Manifest.permission.BLUETOOTH_CONNECT);
            }
        }
        if (!requiredPermissions.isEmpty()) {
            permissionLauncher.launch(requiredPermissions.toArray(new String[0]));
        } else {
            initBluetooth();
        }
    }

    private void initBluetooth() {
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (bluetoothAdapter == null) {
            appendError("Bluetooth is not supported on this device.");
            return;
        }
        if (!bluetoothAdapter.isEnabled()) {
            appendStatus("Bluetooth is currently disabled. Please turn on Bluetooth.");
        } else {
            refreshPairedDevices();
        }
    }

    @SuppressLint("MissingPermission")
    private void refreshPairedDevices() {
        if (bluetoothAdapter == null) return;

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
                ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT)
                        != PackageManager.PERMISSION_GRANTED) {
            appendError("Missing BLUETOOTH_CONNECT permission.");
            return;
        }

        pairedDevices.clear();
        pairedDisplayNames.clear();

        Set<BluetoothDevice> bonded = bluetoothAdapter.getBondedDevices();
        int hc05Index = -1;

        if (bonded != null && !bonded.isEmpty()) {
            int index = 0;
            for (BluetoothDevice device : bonded) {
                pairedDevices.add(device);
                String name = device.getName() != null ? device.getName() : "Unknown";
                String addr = device.getAddress();
                pairedDisplayNames.add(name + " (" + addr + ")");

                String upper = name.toUpperCase(Locale.ROOT);
                if (upper.contains("HC-05") || upper.equals("HC05") || upper.startsWith("HC-") ||
                        addr.equalsIgnoreCase("D3:34:81:F8:14:BD")) {
                    hc05Index = index;
                }
                index++;
            }
        }

        if (pairedDisplayNames.isEmpty()) {
            pairedDisplayNames.add("No paired devices found");
            appendStatus("No paired devices found. Pair HC-05 in Phone Settings first.");
        }

        deviceSpinnerAdapter.notifyDataSetChanged();

        if (hc05Index >= 0) {
            spinnerPairedDevices.setSelection(hc05Index, false);
            BluetoothDevice dev = pairedDevices.get(hc05Index);
            String devName = dev.getName() != null ? dev.getName() : dev.getAddress();
            appendStatus("Selected HC-05: " + devName + " [" + dev.getAddress() + "]");
        }

        btnConnectToggle.setEnabled(!pairedDevices.isEmpty());
    }

    /*
     * Connection Pipeline: Supports both Classic SPP and BLE GATT (Kai Morich Dual Architecture)
     */
    @SuppressLint("MissingPermission")
    private void connect(@NonNull BluetoothDevice device) {
        if (connected || pendingConnect) return;

        pendingConnect = true;
        userRequestedDisconnect = false;
        lastConnectedDevice = device;
        String name = device.getName() != null ? device.getName() : device.getAddress();
        int type = device.getType();

        updateUiState(ConnectionState.CONNECTING, name);
        appendStatus("Connecting to " + name + " [" + device.getAddress() + "]...");

        if (type == BluetoothDevice.DEVICE_TYPE_LE) {
            appendStatus("Detected BLE device. Connecting via Bluetooth LE GATT...");
            connectBle(device, name);
        } else {
            connectClassic(device, name);
        }
    }

    @SuppressLint("MissingPermission")
    private void connectClassic(BluetoothDevice device, String name) {
        executor.execute(() -> {
            try {
                if (bluetoothAdapter != null && bluetoothAdapter.isDiscovering()) {
                    bluetoothAdapter.cancelDiscovery();
                }
            } catch (Exception ignored) {}

            BluetoothSocket activeSocket = null;
            boolean success = false;
            Exception connectException = null;

            try {
                activeSocket = device.createRfcommSocketToServiceRecord(BLUETOOTH_SPP);
                activeSocket.connect();
                success = true;
            } catch (Exception e1) {
                connectException = e1;
                closeSocketSilently(activeSocket);
                activeSocket = null;

                try {
                    activeSocket = device.createInsecureRfcommSocketToServiceRecord(BLUETOOTH_SPP);
                    activeSocket.connect();
                    success = true;
                } catch (Exception e2) {
                    connectException = e2;
                    closeSocketSilently(activeSocket);
                    activeSocket = null;

                    try {
                        Method m = device.getClass().getMethod("createRfcommSocket", int.class);
                        activeSocket = (BluetoothSocket) m.invoke(device, 1);
                        if (activeSocket != null) {
                            activeSocket.connect();
                            success = true;
                        }
                    } catch (Exception e3) {
                        connectException = e3;
                        closeSocketSilently(activeSocket);
                        activeSocket = null;
                    }
                }
            }

            if (success && activeSocket != null && activeSocket.isConnected()) {
                classicSocket = activeSocket;
                try {
                    classicInputStream = classicSocket.getInputStream();
                    classicOutputStream = classicSocket.getOutputStream();
                    connected = true;
                    pendingConnect = false;
                    reconnectAttempts = 0;
                    userRequestedDisconnect = false;

                    mainHandler.post(() -> {
                        updateUiState(ConnectionState.CONNECTED, name);
                        appendStatus("Connected to " + name + " via Classic SPP RFCOMM.");
                        Toast.makeText(MainActivity.this, "✓ Connected to " + name, Toast.LENGTH_SHORT).show();
                        switchMode(currentMode);
                    });

                    runClassicReaderLoop();
                } catch (IOException e) {
                    connected = false;
                    pendingConnect = false;
                    closeSocketSilently(classicSocket);
                    mainHandler.post(() -> {
                        updateUiState(ConnectionState.DISCONNECTED, null);
                        appendError("Socket stream error: " + e.getMessage());
                    });
                }
            } else {
                final String err = (connectException != null ? connectException.getMessage() : "unknown");
                mainHandler.post(() -> appendStatus("Classic SPP failed (" + err + "). Trying BLE GATT fallback..."));
                connectBle(device, name);
            }
        });
    }

    @SuppressLint("MissingPermission")
    private void connectBle(BluetoothDevice device, String name) {
        mainHandler.post(() -> {
            bleGatt = device.connectGatt(this, false, bleCallback, BluetoothDevice.TRANSPORT_LE);
        });
    }

    private final BluetoothGattCallback bleCallback = new BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                pendingConnect = false;
                connected = true;
                mainHandler.post(() -> {
                    updateUiState(ConnectionState.CONNECTED, gatt.getDevice().getName());
                    appendStatus("BLE GATT Connected. Discovering services...");
                });
                gatt.discoverServices();
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                connected = false;
                pendingConnect = false;
                mainHandler.post(() -> {
                    updateUiState(ConnectionState.DISCONNECTED, null);
                    appendStatus("BLE Disconnected.");
                });
                if (bleGatt != null) {
                    try {
                        bleGatt.close();
                    } catch (Exception ignored) {}
                    bleGatt = null;
                }
            }
        }

        @SuppressLint("MissingPermission")
        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                mainHandler.post(() -> appendError("Service discovery failed: " + status));
                return;
            }

            BluetoothGattService service = gatt.getService(BLE_CC254X_SERVICE);
            if (service != null) {
                bleWriteChar = service.getCharacteristic(BLE_CC254X_CHAR);
                bleReadChar = bleWriteChar;
            } else {
                service = gatt.getService(BLE_NRF_SERVICE);
                if (service != null) {
                    bleWriteChar = service.getCharacteristic(BLE_NRF_CHAR_WRITE);
                    bleReadChar = service.getCharacteristic(BLE_NRF_CHAR_READ);
                }
            }

            if (bleReadChar != null) {
                gatt.setCharacteristicNotification(bleReadChar, true);
                BluetoothGattDescriptor desc = bleReadChar.getDescriptor(BLE_CCCD);
                if (desc != null) {
                    desc.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                    gatt.writeDescriptor(desc);
                }
                mainHandler.post(() -> {
                    appendStatus("Serial BLE Service ready.");
                    switchMode(currentMode);
                });
            } else {
                mainHandler.post(() -> appendStatus("Connected, but no standard serial service found."));
            }
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic) {
            byte[] data = characteristic.getValue();
            if (data != null && data.length > 0) {
                String text = new String(data, StandardCharsets.UTF_8);
                mainHandler.post(() -> appendRawRx(text));
            }
        }
    };

    /**
     * Classic streaming reader loop (Kai Morich pattern):
     * Does NOT wait for newlines; immediately sends raw byte chunks to the UI terminal!
     */
    private void runClassicReaderLoop() {
        byte[] buffer = new byte[1024];
        int len;

        while (connected && classicInputStream != null) {
            try {
                len = classicInputStream.read(buffer);
                if (len > 0) {
                    byte[] data = Arrays.copyOf(buffer, len);
                    String textChunk = new String(data, StandardCharsets.UTF_8);
                    mainHandler.post(() -> appendRawRx(textChunk));
                }
            } catch (IOException e) {
                if (connected) {
                    mainHandler.post(() -> {
                        appendStatus("Connection lost: " + e.getMessage());
                        disconnect(false);
                    });
                }
                break;
            }
        }
    }

    @SuppressLint("MissingPermission")
    private void disconnect(boolean isUserRequested) {
        if (!connected && !pendingConnect) return;

        if (isUserRequested) {
            userRequestedDisconnect = true;
            mainHandler.removeCallbacks(autoReconnectRunnable);
        }

        connected = false;
        pendingConnect = false;

        executor.execute(() -> {
            synchronized (socketLock) {
                closeSocketSilently(classicSocket);
                classicSocket = null;
                classicInputStream = null;
                classicOutputStream = null;
            }

            if (bleGatt != null) {
                try {
                    bleGatt.disconnect();
                    bleGatt.close();
                } catch (Exception ignored) {}
                bleGatt = null;
                bleWriteChar = null;
                bleReadChar = null;
            }

            mainHandler.post(() -> {
                updateUiState(ConnectionState.DISCONNECTED, null);
                if (isUserRequested) {
                    appendStatus("Disconnected by user.");
                } else {
                    appendStatus("Connection lost unexpectedly.");
                    if (!userRequestedDisconnect && lastConnectedDevice != null && reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
                        appendStatus("⚡ Attempting auto-reconnect in 1.8s...");
                        mainHandler.postDelayed(autoReconnectRunnable, 1800);
                    }
                }
            });
        });
    }

    private void closeSocketSilently(BluetoothSocket s) {
        if (s != null) {
            try {
                s.close();
            } catch (Exception ignored) {}
        }
    }

    /*
     * UI & Terminal Output Helpers
     */
    private void trimTextViewIfNeeded(TextView tv) {
        if (tv == null) return;
        CharSequence cs = tv.getText();
        if (cs != null && cs.length() > MAX_TERMINAL_CHARS) {
            int cutIndex = cs.length() - TRIM_TARGET_CHARS;
            int nextNewline = cs.toString().indexOf('\n', cutIndex);
            if (nextNewline != -1 && nextNewline < cs.length()) {
                cutIndex = nextNewline + 1;
            }
            tv.setText(cs.subSequence(cutIndex, cs.length()));
        }
    }

    private void appendRawRx(String text) {
        trimTextViewIfNeeded(tvTerminal);
        SpannableString span = new SpannableString(text);
        span.setSpan(new ForegroundColorSpan(COLOR_RX), 0, text.length(), Spannable.SPAN_EXCLUSIVE_EXCLUSIVE);
        tvTerminal.append(span);
        scrollToBottom();

        if (tvObstacleTerminal != null) {
            trimTextViewIfNeeded(tvObstacleTerminal);
            SpannableString obsSpan = new SpannableString(text);
            obsSpan.setSpan(new ForegroundColorSpan(COLOR_RX), 0, text.length(), Spannable.SPAN_EXCLUSIVE_EXCLUSIVE);
            tvObstacleTerminal.append(obsSpan);
            if (scrollObstacleTerminal != null) {
                scrollObstacleTerminal.post(() -> scrollObstacleTerminal.fullScroll(View.FOCUS_DOWN));
            }
        }

        parseArduinoTelemetry(text);
    }

    private void parseArduinoTelemetry(String text) {
        if (text == null) return;
        rxLineBuffer.append(text);

        if (rxLineBuffer.length() > 4096) {
            rxLineBuffer.delete(0, rxLineBuffer.length() - 1024);
        }

        int newlineIdx;
        while ((newlineIdx = rxLineBuffer.indexOf("\n")) != -1) {
            String line = rxLineBuffer.substring(0, newlineIdx).trim();
            rxLineBuffer.delete(0, newlineIdx + 1);
            if (!line.isEmpty()) {
                processTelemetryLine(line);
            }
        }
    }

    private void processTelemetryLine(String line) {
        String upper = line.toUpperCase(Locale.ROOT);

        if (upper.contains("OBSTACLE")) {
            if (tvObstacleStatus != null) {
                tvObstacleStatus.setText("⚠️ Obstacle Detected! Sweeping Left/Right...");
                tvObstacleStatus.setTextColor(ContextCompat.getColor(this, R.color.warning));
            }
            if (tvObstacleSubtext != null) {
                tvObstacleSubtext.setText("Ultrasonic < 15cm • Servo scanning Left (150°) & Right (30°)");
            }
            if (tvObstacleBadge != null) {
                tvObstacleBadge.setText("AVOIDING");
            }
        } else if (upper.contains("TURN_LEFT")) {
            if (tvObstacleStatus != null) {
                tvObstacleStatus.setText("◄ Left Path Clear — Turning Left (600ms)");
                tvObstacleStatus.setTextColor(ContextCompat.getColor(this, R.color.accent));
            }
            if (tvObstacleSubtext != null) {
                tvObstacleSubtext.setText("Left clearance > Right clearance (>15cm) • Executing turn");
            }
            if (tvObstacleBadge != null) {
                tvObstacleBadge.setText("TURNING");
            }
        } else if (upper.contains("TURN_RIGHT")) {
            if (tvObstacleStatus != null) {
                tvObstacleStatus.setText("► Right Path Clear — Turning Right (600ms)");
                tvObstacleStatus.setTextColor(ContextCompat.getColor(this, R.color.accent));
            }
            if (tvObstacleSubtext != null) {
                tvObstacleSubtext.setText("Right clearance > 15cm • Executing turn");
            }
            if (tvObstacleBadge != null) {
                tvObstacleBadge.setText("TURNING");
            }
        } else if (upper.contains("BOTH_BLOCKED")) {
            if (tvObstacleStatus != null) {
                tvObstacleStatus.setText("⛔ Both Sides Blocked! Reversing & Turning Right");
                tvObstacleStatus.setTextColor(ContextCompat.getColor(this, R.color.danger));
            }
            if (tvObstacleSubtext != null) {
                tvObstacleSubtext.setText("Back 500ms + Turn Right 900ms to escape dead-end");
            }
            if (tvObstacleBadge != null) {
                tvObstacleBadge.setText("BLOCKED");
            }
        } else if (upper.contains("MODE:OBSTACLE")) {
            if (tvObstacleStatus != null) {
                tvObstacleStatus.setText("● Forward Path Clear — Navigating");
                tvObstacleStatus.setTextColor(ContextCompat.getColor(this, R.color.status_connected));
            }
            if (tvObstacleSubtext != null) {
                tvObstacleSubtext.setText("Threshold: 15cm • HC-SR04 Active • Center (90°)");
            }
            if (tvObstacleBadge != null) {
                tvObstacleBadge.setText("ACTIVE");
            }
        } else if (upper.contains("ROVER_READY")) {
            if (tvObstacleStatus != null && currentMode == RoverMode.OBSTACLE) {
                tvObstacleStatus.setText("● Rover Ready & Online");
                tvObstacleStatus.setTextColor(ContextCompat.getColor(this, R.color.status_connected));
            }
        } else if (upper.contains("STOP")) {
            if (currentMode == RoverMode.OBSTACLE) {
                if (tvObstacleStatus != null) {
                    tvObstacleStatus.setText("■ Rover Stopped");
                    tvObstacleStatus.setTextColor(ContextCompat.getColor(this, R.color.danger));
                }
                if (tvObstacleBadge != null) {
                    tvObstacleBadge.setText("STOPPED");
                }
            }
        } else if (upper.startsWith("PATH_STEP:")) {
            String stepInfo = line.substring(10).trim();
            if (tvPathStatusBadge != null) {
                tvPathStatusBadge.setText("STEP: " + stepInfo);
                tvPathStatusBadge.setTextColor(ContextCompat.getColor(this, R.color.primary));
            }
        } else if (upper.contains("PATH_COMPLETE")) {
            if (tvPathStatusBadge != null) {
                tvPathStatusBadge.setText("COMPLETED");
                tvPathStatusBadge.setTextColor(ContextCompat.getColor(this, R.color.status_connected));
            }
            Toast.makeText(this, "✅ Path Execution Complete!", Toast.LENGTH_SHORT).show();
            if (tvPathPreview != null) {
                tvPathPreview.setText("✅ Path finished executing successfully!");
            }
        } else if (upper.startsWith("PATH_OBSTACLE")) {
            if (tvPathStatusBadge != null) {
                tvPathStatusBadge.setText("BLOCKED");
                tvPathStatusBadge.setTextColor(ContextCompat.getColor(this, R.color.danger));
            }
            Toast.makeText(this, "⚠️ Obstacle detected! Rover stopped safely.", Toast.LENGTH_LONG).show();
            Toast.makeText(this, "✓ Path Execution Complete!", Toast.LENGTH_SHORT).show();
            if (tvPathPreview != null) {
                tvPathPreview.setText("✓ Path finished executing by rover!");
            }
        }

        try {
            Matcher m = TELEMETRY_DIST_PATTERN.matcher(line);
            if (m.find()) {
                String numStr = m.group(1);
                if (numStr != null) {
                    int dist = Integer.parseInt(numStr);
                    if (dist > 0 && dist < 450) {
                        if (tvObstacleSubtext != null && currentMode == RoverMode.OBSTACLE) {
                            tvObstacleSubtext.setText("Live Distance: " + dist + " cm • Threshold: 15cm");
                        }
                    }
                }
            }
        } catch (Exception ignored) {}
    }

    private void appendTx(String text) {
        trimTextViewIfNeeded(tvTerminal);
        String line = "\n> " + text + "\n";
        SpannableString span = new SpannableString(line);
        span.setSpan(new ForegroundColorSpan(COLOR_TX), 0, line.length(), Spannable.SPAN_EXCLUSIVE_EXCLUSIVE);
        tvTerminal.append(span);
        scrollToBottom();

        if (tvObstacleTerminal != null && currentMode == RoverMode.OBSTACLE) {
            trimTextViewIfNeeded(tvObstacleTerminal);
            SpannableString obsSpan = new SpannableString(line);
            obsSpan.setSpan(new ForegroundColorSpan(COLOR_TX), 0, line.length(), Spannable.SPAN_EXCLUSIVE_EXCLUSIVE);
            tvObstacleTerminal.append(obsSpan);
            if (scrollObstacleTerminal != null) {
                scrollObstacleTerminal.post(() -> scrollObstacleTerminal.fullScroll(View.FOCUS_DOWN));
            }
        }
    }

    private void appendStatus(String text) {
        trimTextViewIfNeeded(tvTerminal);
        String timestamp = timeFormat.format(new Date());
        String line = "[" + timestamp + "] " + text + "\n";
        SpannableString span = new SpannableString(line);
        span.setSpan(new ForegroundColorSpan(COLOR_STATUS), 0, line.length(), Spannable.SPAN_EXCLUSIVE_EXCLUSIVE);
        tvTerminal.append(span);
        scrollToBottom();

        if (tvObstacleTerminal != null && currentMode == RoverMode.OBSTACLE) {
            trimTextViewIfNeeded(tvObstacleTerminal);
            SpannableString obsSpan = new SpannableString(line);
            obsSpan.setSpan(new ForegroundColorSpan(COLOR_STATUS), 0, line.length(), Spannable.SPAN_EXCLUSIVE_EXCLUSIVE);
            tvObstacleTerminal.append(obsSpan);
            if (scrollObstacleTerminal != null) {
                scrollObstacleTerminal.post(() -> scrollObstacleTerminal.fullScroll(View.FOCUS_DOWN));
            }
        }
    }

    private void appendError(String text) {
        trimTextViewIfNeeded(tvTerminal);
        String timestamp = timeFormat.format(new Date());
        String line = "[" + timestamp + "] " + text + "\n";
        SpannableString span = new SpannableString(line);
        span.setSpan(new ForegroundColorSpan(COLOR_ERROR), 0, line.length(), Spannable.SPAN_EXCLUSIVE_EXCLUSIVE);
        tvTerminal.append(span);
        scrollToBottom();

        if (tvObstacleTerminal != null && currentMode == RoverMode.OBSTACLE) {
            trimTextViewIfNeeded(tvObstacleTerminal);
            SpannableString obsSpan = new SpannableString(line);
            obsSpan.setSpan(new ForegroundColorSpan(COLOR_ERROR), 0, line.length(), Spannable.SPAN_EXCLUSIVE_EXCLUSIVE);
            tvObstacleTerminal.append(obsSpan);
            if (scrollObstacleTerminal != null) {
                scrollObstacleTerminal.post(() -> scrollObstacleTerminal.fullScroll(View.FOCUS_DOWN));
            }
        }
    }

    private void scrollToBottom() {
        if (scrollTerminal != null) {
            scrollTerminal.post(() -> scrollTerminal.fullScroll(View.FOCUS_DOWN));
        }
    }

    private void updateUiState(ConnectionState state, String deviceName) {
        GradientDrawable badgeBg = new GradientDrawable();
        badgeBg.setCornerRadius(dpToPx(20));

        switch (state) {
            case CONNECTED:
                tvConnectionBadge.setText("● Connected to HC-05");
                tvConnectionBadge.setTextColor(ContextCompat.getColor(this, R.color.status_connected));
                badgeBg.setColor(ContextCompat.getColor(this, R.color.status_connected_bg));
                badgeBg.setStroke(dpToPx(1), ContextCompat.getColor(this, R.color.status_connected));
                tvConnectionBadge.setBackground(badgeBg);

                btnConnectToggle.setText("Disconnect");
                btnConnectToggle.setBackgroundTintList(ContextCompat.getColorStateList(this, R.color.status_disconnected));
                btnConnectToggle.setEnabled(true);
                spinnerPairedDevices.setEnabled(false);
                btnRefresh.setEnabled(false);
                if (tvObstacleBadge != null) {
                    tvObstacleBadge.setText(currentMode == RoverMode.OBSTACLE ? "STANDBY" : "READY");
                }
                setControlsEnabled(true);
                break;

            case CONNECTING:
                tvConnectionBadge.setText("◌ Connecting...");
                tvConnectionBadge.setTextColor(ContextCompat.getColor(this, R.color.status_connecting));
                badgeBg.setColor(ContextCompat.getColor(this, R.color.status_connecting_bg));
                badgeBg.setStroke(dpToPx(1), ContextCompat.getColor(this, R.color.status_connecting));
                tvConnectionBadge.setBackground(badgeBg);

                btnConnectToggle.setText("Cancel");
                btnConnectToggle.setBackgroundTintList(ContextCompat.getColorStateList(this, R.color.status_connecting));
                btnConnectToggle.setEnabled(true);
                spinnerPairedDevices.setEnabled(false);
                btnRefresh.setEnabled(false);
                if (tvObstacleBadge != null) {
                    tvObstacleBadge.setText("CONNECTING");
                }
                setControlsEnabled(false);
                break;

            case DISCONNECTED:
            default:
                tvConnectionBadge.setText("○ Disconnected");
                tvConnectionBadge.setTextColor(ContextCompat.getColor(this, R.color.status_disconnected));
                badgeBg.setColor(ContextCompat.getColor(this, R.color.status_disconnected_bg));
                badgeBg.setStroke(dpToPx(1), ContextCompat.getColor(this, R.color.status_disconnected));
                tvConnectionBadge.setBackground(badgeBg);

                btnConnectToggle.setText("Connect");
                btnConnectToggle.setBackgroundTintList(ContextCompat.getColorStateList(this, R.color.primary));
                btnConnectToggle.setEnabled(!pairedDevices.isEmpty());
                spinnerPairedDevices.setEnabled(true);
                btnRefresh.setEnabled(true);
                if (tvObstacleBadge != null) {
                    tvObstacleBadge.setText("OFFLINE");
                }
                if (tvObstacleStatus != null) {
                    tvObstacleStatus.setText("○ Rover Disconnected");
                    tvObstacleStatus.setTextColor(ContextCompat.getColor(this, R.color.status_disconnected));
                }
                setControlsEnabled(false);
                break;
        }
    }

    private void setControlsEnabled(boolean enabled) {
        if (bannerDisconnected != null) {
            bannerDisconnected.setVisibility(enabled ? View.GONE : View.VISIBLE);
        }

        if (btnManualFwd != null) btnManualFwd.setEnabled(enabled);
        if (btnManualBack != null) btnManualBack.setEnabled(enabled);
        if (btnManualLeft != null) btnManualLeft.setEnabled(enabled);
        if (btnManualRight != null) btnManualRight.setEnabled(enabled);
        if (btnManualStop != null) btnManualStop.setEnabled(enabled);
        if (sbSpeed != null) sbSpeed.setEnabled(enabled);
        if (btnSpeed80 != null) btnSpeed80.setEnabled(enabled);
        if (btnSpeed120 != null) btnSpeed120.setEnabled(enabled);
        if (btnSpeed180 != null) btnSpeed180.setEnabled(enabled);
        if (btnSpeed255 != null) btnSpeed255.setEnabled(enabled);

        if (btnObstacleStart != null) btnObstacleStart.setEnabled(enabled);
        if (btnObstacleStop != null) btnObstacleStop.setEnabled(enabled);

        if (btnSendPath != null) btnSendPath.setEnabled(enabled);
        if (btnPathStop != null) btnPathStop.setEnabled(enabled);

        if (btnSend != null) btnSend.setEnabled(enabled);
    }

    private float cachedDensity = 0f;

    private void registerDisconnectReceiver() {
        IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_ACL_DISCONNECTED);
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.TIRAMISU) {
            registerReceiver(aclDisconnectReceiver, filter, Context.RECEIVER_NOT_EXPORTED);
        } else {
            registerReceiver(aclDisconnectReceiver, filter);
        }
    }

    private int dpToPx(int dp) {
        if (cachedDensity <= 0f) {
            cachedDensity = getResources().getDisplayMetrics().density;
        }
        return Math.round(dp * cachedDensity);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        try {
            unregisterReceiver(aclDisconnectReceiver);
        } catch (Exception ignored) {}
        mainHandler.removeCallbacks(autoReconnectRunnable);
        disconnect(true);
        executor.shutdownNow();
        sendExecutor.shutdownNow();
    }

    private enum ConnectionState {
        DISCONNECTED,
        CONNECTING,
        CONNECTED
    }
}
