package de.kai_morich.simple_bluetooth_le_terminal.fragments;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.IBinder;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;

import de.kai_morich.simple_bluetooth_le_terminal.R;
import de.kai_morich.simple_bluetooth_le_terminal.SerialListener;
import de.kai_morich.simple_bluetooth_le_terminal.SerialService;
import de.kai_morich.simple_bluetooth_le_terminal.SerialSocket;

public abstract class SerialEnabledFragment extends Fragment implements ServiceConnection, SerialListener {

    private static final String TAG = SerialEnabledFragment.class.getSimpleName();

    protected enum ConnectionStatus {
        CONNECTED,
        PENDING,
        NOT_CONNECTED
    }

    protected String deviceAddress;
    protected SerialService service;
    private ConnectionStatus connectionStatus;
    private boolean initialStart = true;

    @Override
    public void onDestroy() {
        if (connectionStatus != ConnectionStatus.NOT_CONNECTED) {
            disconnect();
        }
        getActivity().stopService(new Intent(getActivity(), SerialService.class));
        super.onDestroy();
    }

    @Override
    public void onStart() {
        super.onStart();
        if (service != null) {
            service.attach(this);
        } else {
            getActivity().startService(new Intent(getActivity(), SerialService.class)); // prevents service destroy on unbind from recreated activity caused by orientation change
        }
    }

    @Override
    public void onStop() {
        if (service != null && !getActivity().isChangingConfigurations()) {
            service.detach();
        }
        super.onStop();
    }

    @SuppressWarnings("deprecation")
    // onAttach(context) was added with API 23. onAttach(activity) works for all API versions
    @Override
    public void onAttach(@NonNull Activity activity) {
        super.onAttach(activity);
        getActivity().bindService(new Intent(getActivity(), SerialService.class), this, Context.BIND_AUTO_CREATE);
    }

    @Override
    public void onDetach() {
        try {
            getActivity().unbindService(this);
        } catch (Exception ignored) {
        }
        super.onDetach();
    }

    @Override
    public void onResume() {
        super.onResume();
        getActivity().setTitle(R.string.terminal_title);
        if (initialStart && service != null) {
            initialStart = false;
            getActivity().runOnUiThread(this::connect);
        }
    }

    @Override
    public void onServiceConnected(ComponentName name, IBinder binder) {
        Log.i(TAG, "onServiceConnected");

        service = ((SerialService.SerialBinder) binder).getService();
        service.attach(this);
        if(initialStart && isResumed()) {
            initialStart = false;
            getActivity().runOnUiThread(this::connect);
        }
    }

    @Override
    public void onServiceDisconnected(ComponentName name) {
        service = null;
    }

    protected void connect() {
        Log.i(TAG, "connect()");
        try {
            BluetoothAdapter bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
            BluetoothDevice device = bluetoothAdapter.getRemoteDevice(deviceAddress);
            connectionStatus = ConnectionStatus.PENDING;
            SerialSocket socket = new SerialSocket(getActivity().getApplicationContext(), device);
            service.connect(socket);
        } catch (Exception e) {
            onSerialConnectError(e);
        }
    }

    protected void disconnect() {
        Log.i(TAG, "disconnect()");
        connectionStatus = ConnectionStatus.NOT_CONNECTED;
        service.disconnect();
    }

    @Override
    public void onSerialConnect() {
        Log.i(TAG, "onSerialConnect(): connected");
        connectionStatus = ConnectionStatus.CONNECTED;
    }

    @Override
    public void onSerialConnectError(Exception e) {
        disconnect();
    }

    @Override
    public void onSerialIoError(Exception e) {
        disconnect();
    }

    public boolean isConnected() {
        return connectionStatus == ConnectionStatus.CONNECTED;
    }
}
