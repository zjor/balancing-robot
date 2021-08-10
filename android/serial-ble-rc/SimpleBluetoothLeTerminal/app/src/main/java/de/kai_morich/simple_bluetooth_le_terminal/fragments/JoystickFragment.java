package de.kai_morich.simple_bluetooth_le_terminal.fragments;

import android.os.Bundle;
import android.util.Log;
import android.util.Pair;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

import java.util.concurrent.TimeUnit;

import de.kai_morich.simple_bluetooth_le_terminal.R;
import de.kai_morich.simple_bluetooth_le_terminal.views.JoystickView;
import io.reactivex.rxjava3.android.schedulers.AndroidSchedulers;
import io.reactivex.rxjava3.disposables.Disposable;
import io.reactivex.rxjava3.schedulers.Schedulers;
import io.reactivex.rxjava3.subjects.PublishSubject;

public class JoystickFragment extends Fragment implements JoystickView.JoystickListener {

    public static final float VELOCITY_MAX = 10.0f;
    public static final float STEERING_MAX = 2.0f;

    private TextView velocityView;
    private TextView steeringView;

    private PublishSubject<Pair<Float, Float>> joystickStream = PublishSubject.create();
    private Disposable subscription;

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
    public void onResume() {
        super.onResume();
        subscription = joystickStream.subscribeOn(Schedulers.newThread())
                .observeOn(AndroidSchedulers.mainThread())
                .debounce(50, TimeUnit.MILLISECONDS)
                .subscribe((pair) -> Log.i("xx", "Pair arrived: " + pair));
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
}
