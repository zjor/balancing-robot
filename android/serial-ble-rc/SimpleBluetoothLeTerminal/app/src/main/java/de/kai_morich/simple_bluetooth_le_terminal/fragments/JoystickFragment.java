package de.kai_morich.simple_bluetooth_le_terminal.fragments;

import android.os.Bundle;
import android.util.Log;
import android.util.Pair;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

import de.kai_morich.simple_bluetooth_le_terminal.R;
import de.kai_morich.simple_bluetooth_le_terminal.TextUtil;
import de.kai_morich.simple_bluetooth_le_terminal.views.JoystickView;
import io.reactivex.rxjava3.android.schedulers.AndroidSchedulers;
import io.reactivex.rxjava3.disposables.Disposable;
import io.reactivex.rxjava3.schedulers.Schedulers;
import io.reactivex.rxjava3.subjects.PublishSubject;

import static de.kai_morich.simple_bluetooth_le_terminal.fragments.PIDSettingsFragment.DIVISOR;

public class JoystickFragment extends SerialEnabledFragment implements JoystickView.JoystickListener {

    public static final String TAG = JoystickFragment.class.getSimpleName();

    public static final float VELOCITY_MAX = 10.0f;
    public static final float STEERING_MAX = 2.0f;

    private TextView velocityView;
    private TextView steeringView;

    private PublishSubject<Pair<Float, Float>> joystickStream = PublishSubject.create();
    private Disposable subscription;

    @Override
    public void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setHasOptionsMenu(true);
        deviceAddress = getArguments().getString("device");
    }

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_joystick, container, false);
        velocityView = view.findViewById(R.id.velocity);
        steeringView = view.findViewById(R.id.steering);
        ((JoystickView) view.findViewById(R.id.joystick_view)).setJoystickCallback(this);
        return view;
    }

    @Override
    public void onCreateOptionsMenu(@NonNull Menu menu, MenuInflater inflater) {
        inflater.inflate(R.menu.menu_joystick, menu);
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();
        if (id == R.id.pid_settings) {
            disconnect();
            PIDSettingsFragment fragment = new PIDSettingsFragment();

            Bundle args = new Bundle();
            args.putString("device", deviceAddress);
            fragment.setArguments(args);

            getParentFragmentManager().beginTransaction()
                    .replace(R.id.fragment, fragment, PIDSettingsFragment.FRAGMENT_TAG)
                    .addToBackStack(null)
                    .commit();
            return true;
        } else {
            return super.onOptionsItemSelected(item);
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        getActivity().setTitle(R.string.joystick_title);
        subscription = joystickStream.subscribeOn(Schedulers.newThread())
                .observeOn(AndroidSchedulers.mainThread())
                .throttleLast(100, TimeUnit.MILLISECONDS)
                .subscribe((pair) -> sendJoystickState(pair.first, pair.second));
    }

    @Override
    public void onPause() {
        super.onPause();
        if (subscription != null) {
            subscription.dispose();
        }
    }

    @Override
    public void onJoystickMoved(float xPercent, float yPercent, int id) {
        final float velocity = -yPercent * VELOCITY_MAX;
        final float steering = xPercent * STEERING_MAX;

        velocityView.setText(String.format("Velocity: %.2f", velocity));
        steeringView.setText(String.format("Steering: %.2f", steering));
        joystickStream.onNext(Pair.create(velocity, steering));
    }

    @Override
    public void onSerialConnect() {
        super.onSerialConnect();
        Toast.makeText(getActivity(), "Connected", Toast.LENGTH_SHORT).show();
    }

    @Override
    public void onSerialRead(byte[] data) {
        Log.i(TAG, "onSerialRead: " + new String(data));
    }

    private void sendJoystickState(float velocity, float steering) {
        if (service != null) {
            if (!isConnected()) {
                return;
            }
            StringBuilder command = new StringBuilder("c")
                    .append((int) (velocity * DIVISOR))
                    .append(';')
                    .append((int) (steering * DIVISOR))
                    .append(TextUtil.newline_crlf);
            try {
                service.write(command.toString().getBytes());
            } catch (IOException e) {
                onSerialIoError(e);
            }
        }
    }
}
