package de.kai_morich.simple_bluetooth_le_terminal.fragments;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

import java.util.ArrayList;
import java.util.List;

import de.kai_morich.simple_bluetooth_le_terminal.R;

public class PIDSettingsFragment extends Fragment {

    public static final String FRAGMENT_TAG = "pid_settings";

    private List<TextView> pidSettings = new ArrayList<>();

    private Button cancelButton;
    private Button saveButton;

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
        applyPIDSettings(new double[] {450., 20.0, .0, 0.005, 0.2, 0.0});
    }

    private void applyPIDSettings(double[] settings) {
        for (int i = 0; i < settings.length; i++) {
            pidSettings.get(i).setText(String.valueOf(settings[i]));
        }
    }

}