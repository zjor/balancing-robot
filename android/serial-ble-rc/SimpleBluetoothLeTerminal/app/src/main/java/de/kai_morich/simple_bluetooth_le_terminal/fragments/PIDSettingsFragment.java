package de.kai_morich.simple_bluetooth_le_terminal.fragments;

import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;

import de.kai_morich.simple_bluetooth_le_terminal.R;
import de.kai_morich.simple_bluetooth_le_terminal.TextUtil;

public class PIDSettingsFragment extends SerialEnabledFragment {

    private static final String TAG = PIDSettingsFragment.class.getSimpleName();

    public static final String FRAGMENT_TAG = "pid_settings";

    private static final String REQUEST_PID_SETTINGS_COMMAND = "r" + TextUtil.newline_crlf;
    public static final double DIVISOR = 10000.0;

    private static final Pattern SETTINGS_PACKET_REGEX = Pattern.compile("^(\\-?\\d+)+;(\\-?\\d+)+;(\\-?\\d+)+;(\\-?\\d+)+;(\\-?\\d+)+;(\\-?\\d+)$");

    private List<EditText> pidSettings = new ArrayList<>();

    private Button cancelButton;
    private Button saveButton;

    private byte[] packet = new byte[1024];
    private int packetSize = 0;

    @Override
    public void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        deviceAddress = getArguments().getString("device");
    }

    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_pid_settings, container, false);
        int[] ids = new int[]{
                R.id.pid_balance_kp,
                R.id.pid_balance_kd,
                R.id.pid_balance_ki,
                R.id.pid_velocity_kp,
                R.id.pid_velocity_kd,
                R.id.pid_velocity_ki,
        };
        for (int id : ids) {
            pidSettings.add((EditText) view.findViewById(id));
        }

        saveButton = (Button) view.findViewById(R.id.pid_settings_button_save);
        saveButton.setOnClickListener(v -> savePIDSettings());
        cancelButton = (Button) view.findViewById(R.id.pid_settings_button_cancel);

        return view;
    }

    @Override
    public void onResume() {
        super.onResume();
        getActivity().setTitle(R.string.pid_settings_title);
        requestPIDSettings();
    }

    private void applyPIDSettings(double[] settings) {
        for (int i = 0; i < settings.length; i++) {
            pidSettings.get(i).setText(String.valueOf(settings[i]));
        }
    }

    @Override
    public void onSerialConnect() {
        super.onSerialConnect();
        Toast.makeText(getActivity(), "Connected", Toast.LENGTH_SHORT).show();
        requestPIDSettings();
    }

    @Override
    public void onSerialRead(byte[] data) {
        for (byte b : data) {
            if (b == '\r') {
                String packetStr = new String(packet, 0, packetSize);
                Log.i(TAG, packetStr);
                handlePacket(packetStr);
                packetSize = 0;
            } else if (b == '\n') {
                continue;
            } else {
                packet[packetSize++] = b;
            }
        }
    }

    private void handlePacket(String packet) {
        if (!SETTINGS_PACKET_REGEX.matcher(packet).matches()) {
            Toast.makeText(getActivity(), "Unrecognized packet pattern: " + packet, Toast.LENGTH_SHORT).show();
            return;
        }
        double[] settings = new double[6];
        String[] split = packet.split(";");
        for (int i = 0; i < split.length; i++) {
            settings[i] = Double.parseDouble(split[i]) / DIVISOR;
        }
        applyPIDSettings(settings);
    }

    private void requestPIDSettings() {
        try {
            if (service != null) {
                service.write(REQUEST_PID_SETTINGS_COMMAND.getBytes());
            } else {
                Log.i("PIDSettings", "service == null");
            }
        } catch (IOException e) {
            onSerialIoError(e);
        }
    }

    private void savePIDSettings() {
        StringBuilder builder = new StringBuilder("s");
        for (EditText text : pidSettings) {
            long value = (long) (Double.parseDouble(text.getText().toString()) * DIVISOR);
            builder.append(value).append(';');
        }
        builder.setLength(builder.length() - 1);
        builder.append(TextUtil.newline_crlf);
        if (service != null) {
            try {
                service.write(builder.toString().getBytes());
            } catch (IOException e) {
                onSerialIoError(e);
            }
        }
        Toast.makeText(getActivity(), "Saved", Toast.LENGTH_SHORT).show();
    }
}