package de.kai_morich.simple_bluetooth_le_terminal.fragments;

import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import de.kai_morich.simple_bluetooth_le_terminal.R;
import de.kai_morich.simple_bluetooth_le_terminal.TextUtil;

public class PIDSettingsFragment extends SerialEnabledFragment {

    private static final String TAG = PIDSettingsFragment.class.getSimpleName();

    public static final String FRAGMENT_TAG = "pid_settings";

    private static final String REQUEST_PID_SETTINGS_COMMAND = "r" + TextUtil.newline_crlf;

    private List<TextView> pidSettings = new ArrayList<>();

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
        int[] ids = new int[] {
                R.id.pid_balance_kp,
                R.id.pid_balance_kd,
                R.id.pid_balance_ki,
                R.id.pid_velocity_kp,
                R.id.pid_velocity_kd,
                R.id.pid_velocity_ki,
        };
        for (int id: ids) {
            pidSettings.add((TextView) view.findViewById(id));
        }

        saveButton = (Button) view.findViewById(R.id.pid_settings_button_save);
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
        Log.i(TAG, new String(data));
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
}