//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.robotcontroller.internal;

import android.app.ActionBar;
import android.app.Activity;
import android.app.ActivityManager;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.res.Configuration;
import android.hardware.usb.UsbDevice;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.Build.VERSION;
import android.preference.PreferenceManager;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.webkit.WebView;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.TextView;
import androidx.annotation.Nullable;
import androidx.annotation.StringRes;
import com.google.blocks.ftcrobotcontroller.ProgrammingWebHandlers;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;
import com.qualcomm.ftccommon.ClassManagerFactory;
import com.qualcomm.ftccommon.FtcAboutActivity;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.FtcEventLoopIdle;
import com.qualcomm.ftccommon.FtcRobotControllerService;
import com.qualcomm.ftccommon.FtcRobotControllerSettingsActivity;
import com.qualcomm.ftccommon.Restarter;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.ftccommon.LaunchActivityConstantsList.RequestCode;
import com.qualcomm.ftccommon.configuration.EditParameters;
import com.qualcomm.ftccommon.configuration.FtcLoadFileActivity;
import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.ftcrobotcontroller.R.id;
import com.qualcomm.ftcrobotcontroller.R.layout;
import com.qualcomm.ftcrobotcontroller.R.menu;
import com.qualcomm.ftcrobotcontroller.R.string;
import com.qualcomm.ftcrobotcontroller.R.xml;
import com.qualcomm.hardware.HardwareFactory;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.ClockWarningSource;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.Dimmer;
import com.qualcomm.robotcore.util.ImmersiveMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.wifi.NetworkType;
import com.qualcomm.robotcore.wifi.NetworkConnection.NetworkEvent;
import java.io.FileNotFoundException;
import java.util.Iterator;
import java.util.List;
import java.util.Objects;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import org.firstinspires.ftc.ftccommon.external.SoundPlayingRobotMonitor;
import org.firstinspires.ftc.ftccommon.internal.AnnotatedHooksClassFilter;
import org.firstinspires.ftc.ftccommon.internal.FtcRobotControllerWatchdogService;
import org.firstinspires.ftc.ftccommon.internal.ProgramAndManageActivity;
import org.firstinspires.ftc.onbotjava.ExternalLibraries;
import org.firstinspires.ftc.onbotjava.OnBotJavaHelperImpl;
import org.firstinspires.ftc.onbotjava.OnBotJavaProgrammingMode;
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.robotcore.internal.hardware.android.AndroidBoard;
import org.firstinspires.ftc.robotcore.internal.network.DeviceNameManagerFactory;
import org.firstinspires.ftc.robotcore.internal.network.PreferenceRemoterRC;
import org.firstinspires.ftc.robotcore.internal.network.StartResult;
import org.firstinspires.ftc.robotcore.internal.network.WifiDirectChannelChanger;
import org.firstinspires.ftc.robotcore.internal.network.WifiMuteEvent;
import org.firstinspires.ftc.robotcore.internal.network.WifiMuteStateMachine;
import org.firstinspires.ftc.robotcore.internal.opmode.ClassManager;
import org.firstinspires.ftc.robotcore.internal.opmode.OnBotJavaHelper;
import org.firstinspires.ftc.robotcore.internal.system.AppAliveNotifier;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.robotcore.internal.system.PreferencesHelper;
import org.firstinspires.ftc.robotcore.internal.system.ServiceController;
import org.firstinspires.ftc.robotcore.internal.ui.ThemedActivity;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;
import org.firstinspires.ftc.robotcore.internal.webserver.RobotControllerWebInfo;
import org.firstinspires.ftc.robotserver.internal.programmingmode.ProgrammingModeManager;
import org.firstinspires.inspection.RcInspectionActivity;
import org.xmlpull.v1.XmlPullParserException;

public class FtcRobotControllerActivity extends Activity {
    public static final String TAG = "RCActivity";
    private static final int REQUEST_CONFIG_WIFI_CHANNEL = 1;
    private static final int NUM_GAMEPADS = 2;
    protected WifiManager.WifiLock wifiLock;
    protected RobotConfigFileManager cfgFileMgr;
    private OnBotJavaHelper onBotJavaHelper;
    protected ProgrammingModeManager programmingModeManager;
    protected UpdateUI.Callback callback;
    protected Context context;
    protected Utility utility;
    protected StartResult prefRemoterStartResult = new StartResult();
    protected StartResult deviceNameStartResult = new StartResult();
    protected PreferencesHelper preferencesHelper;
    protected final SharedPreferencesListener sharedPreferencesListener = new SharedPreferencesListener(this);
    protected ImageButton buttonMenu;
    protected TextView textDeviceName;
    protected TextView textNetworkConnectionStatus;
    protected TextView textRobotStatus;
    protected TextView[] textGamepad = new TextView[2];
    protected TextView textOpMode;
    protected TextView textErrorMessage;
    protected ImmersiveMode immersion;
    protected UpdateUI updateUI;
    protected Dimmer dimmer;
    protected LinearLayout entireScreenLayout;
    protected FtcRobotControllerService controllerService;
    protected NetworkType networkType;
    protected FtcEventLoop eventLoop;
    protected Queue<UsbDevice> receivedUsbAttachmentNotifications;
    protected WifiMuteStateMachine wifiMuteStateMachine;
    protected MotionDetection motionDetection;
    private static boolean permissionsValidated = false;
    private WifiDirectChannelChanger wifiDirectChannelChanger;
    protected boolean serviceShouldUnbind = false;
    protected ServiceConnection connection = new 1(this);

    public FtcRobotControllerActivity() {
    }

    public String getTag() {
        return "RCActivity";
    }

    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        if ("android.hardware.usb.action.USB_DEVICE_ATTACHED".equals(intent.getAction())) {
            UsbDevice usbDevice = (UsbDevice)intent.getParcelableExtra("device");
            RobotLog.vv("RCActivity", "ACTION_USB_DEVICE_ATTACHED: %s", new Object[]{usbDevice.getDeviceName()});
            if (usbDevice != null && this.receivedUsbAttachmentNotifications != null) {
                this.receivedUsbAttachmentNotifications.add(usbDevice);
                this.passReceivedUsbAttachmentsToEventLoop();
            }
        }

    }

    protected void passReceivedUsbAttachmentsToEventLoop() {
        if (this.eventLoop != null) {
            while(true) {
                UsbDevice usbDevice = (UsbDevice)this.receivedUsbAttachmentNotifications.poll();
                if (usbDevice == null) {
                    break;
                }

                this.eventLoop.onUsbDeviceAttached(usbDevice);
            }
        } else {
            while(this.receivedUsbAttachmentNotifications.size() > 100) {
                this.receivedUsbAttachmentNotifications.poll();
            }
        }

    }

    protected boolean enforcePermissionValidator() {
        if (!permissionsValidated) {
            RobotLog.vv("RCActivity", "Redirecting to permission validator");
            Intent permissionValidatorIntent = new Intent(AppUtil.getDefContext(), PermissionValidatorWrapper.class);
            this.startActivity(permissionValidatorIntent);
            this.finish();
            return true;
        } else {
            RobotLog.vv("RCActivity", "Permissions validated already");
            return false;
        }
    }

    public static void setPermissionsValidated() {
        permissionsValidated = true;
    }

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (!this.enforcePermissionValidator()) {
            RobotLog.onApplicationStart();
            RobotLog.vv("RCActivity", "onCreate()");
            ThemedActivity.appAppThemeToActivity(this.getTag(), this);
            RobotLog.vv("RCActivity", "rootActivity is of class %s", new Object[]{AppUtil.getInstance().getRootActivity().getClass().getSimpleName()});
            RobotLog.vv("RCActivity", "launchActivity is of class %s", new Object[]{FtcRobotControllerWatchdogService.launchActivity()});
            Assert.assertTrue(FtcRobotControllerWatchdogService.isLaunchActivity(AppUtil.getInstance().getRootActivity()));
            Assert.assertTrue(AppUtil.getInstance().isRobotController());
            if (LynxConstants.isRevControlHub()) {
                AndroidBoard.getInstance().getAndroidBoardIsPresentPin().setState(true);
            }

            this.context = this;
            this.utility = new Utility(this);
            DeviceNameManagerFactory.getInstance().start(this.deviceNameStartResult);
            PreferenceRemoterRC.getInstance().start(this.prefRemoterStartResult);
            this.receivedUsbAttachmentNotifications = new ConcurrentLinkedQueue();
            this.eventLoop = null;
            this.setContentView(layout.activity_ftc_controller);
            this.preferencesHelper = new PreferencesHelper("RCActivity", this.context);
            this.preferencesHelper.writeBooleanPrefIfDifferent(this.context.getString(string.pref_rc_connected), true);
            this.preferencesHelper.getSharedPreferences().registerOnSharedPreferenceChangeListener(this.sharedPreferencesListener);
            int ftcSeasonYearOfPreviouslyInstalledRc = this.preferencesHelper.readInt(this.getString(string.pref_ftc_season_year_of_current_rc), 0);
            int ftcSeasonYearOfCurrentlyInstalledRc = AppUtil.getInstance().getFtcSeasonYear(AppUtil.getInstance().getLocalSdkBuildMonth()).getValue();
            if (ftcSeasonYearOfCurrentlyInstalledRc > ftcSeasonYearOfPreviouslyInstalledRc) {
                this.preferencesHelper.writeIntPrefIfDifferent(this.getString(string.pref_ftc_season_year_of_current_rc), ftcSeasonYearOfCurrentlyInstalledRc);
                this.preferencesHelper.writeBooleanPrefIfDifferent(this.getString(string.pref_warn_about_2_4_ghz_band), true);
                this.preferencesHelper.writeBooleanPrefIfDifferent(this.getString(string.pref_warn_about_obsolete_software), true);
                this.preferencesHelper.writeBooleanPrefIfDifferent(this.getString(string.pref_warn_about_mismatched_app_versions), true);
                this.preferencesHelper.writeBooleanPrefIfDifferent(this.getString(string.pref_warn_about_incorrect_clocks), true);
            }

            this.entireScreenLayout = (LinearLayout)this.findViewById(id.entire_screen);
            this.buttonMenu = (ImageButton)this.findViewById(id.menu_buttons);
            this.buttonMenu.setOnClickListener(new 2(this));
            this.updateMonitorLayout(this.getResources().getConfiguration());
            BlocksOpMode.setActivityAndWebView(this, (WebView)this.findViewById(id.webViewBlocksRuntime));
            ExternalLibraries.getInstance().onCreate();
            this.onBotJavaHelper = new OnBotJavaHelperImpl();
            if (permissionsValidated) {
                ClassManager.getInstance().setOnBotJavaClassHelper(this.onBotJavaHelper);
                ClassManagerFactory.registerFilters();
                ClassManagerFactory.processAllClasses();
            }

            this.cfgFileMgr = new RobotConfigFileManager(this);
            RobotConfigFile configFile = this.cfgFileMgr.getActiveConfig();
            if (configFile.isDirty()) {
                configFile.markClean();
                this.cfgFileMgr.setActiveConfig(false, configFile);
            }

            this.textDeviceName = (TextView)this.findViewById(id.textDeviceName);
            this.textNetworkConnectionStatus = (TextView)this.findViewById(id.textNetworkConnectionStatus);
            this.textRobotStatus = (TextView)this.findViewById(id.textRobotStatus);
            this.textOpMode = (TextView)this.findViewById(id.textOpMode);
            this.textErrorMessage = (TextView)this.findViewById(id.textErrorMessage);
            this.textGamepad[0] = (TextView)this.findViewById(id.textGamepad1);
            this.textGamepad[1] = (TextView)this.findViewById(id.textGamepad2);
            this.immersion = new ImmersiveMode(this.getWindow().getDecorView());
            this.dimmer = new Dimmer(this);
            this.dimmer.longBright();
            this.programmingModeManager = new ProgrammingModeManager();
            this.programmingModeManager.register(new ProgrammingWebHandlers());
            this.programmingModeManager.register(new OnBotJavaProgrammingMode());
            this.updateUI = this.createUpdateUI();
            this.callback = this.createUICallback(this.updateUI);
            PreferenceManager.setDefaultValues(this, xml.app_settings, false);
            WifiManager wifiManager = (WifiManager)this.getApplicationContext().getSystemService("wifi");
            this.wifiLock = wifiManager.createWifiLock(3, "");
            this.hittingMenuButtonBrightensScreen();
            this.wifiLock.acquire();
            this.callback.networkConnectionUpdate(NetworkEvent.DISCONNECTED);
            this.readNetworkType();
            ServiceController.startService(FtcRobotControllerWatchdogService.class);
            this.bindToService();
            RobotLog.logAppInfo();
            RobotLog.logDeviceInfo();
            AndroidBoard.getInstance().logAndroidBoardInfo();
            if (this.preferencesHelper.readBoolean(this.getString(string.pref_wifi_automute), false)) {
                this.initWifiMute(true);
            }

            FtcAboutActivity.setBuildTimeFromBuildConfig("2024-12-07T17:06:10.635-0500");
            this.checkPreferredChannel();
            AnnotatedHooksClassFilter.getInstance().callOnCreateMethods(this);
        }
    }

    protected UpdateUI createUpdateUI() {
        Restarter restarter = new RobotRestarter(this);
        UpdateUI result = new UpdateUI(this, this.dimmer);
        result.setRestarter(restarter);
        result.setTextViews(this.textNetworkConnectionStatus, this.textRobotStatus, this.textGamepad, this.textOpMode, this.textErrorMessage, this.textDeviceName);
        return result;
    }

    protected UpdateUI.Callback createUICallback(UpdateUI updateUI) {
        Objects.requireNonNull(updateUI);
        UpdateUI.Callback result = new UpdateUI.Callback(updateUI);
        result.setStateMonitor(new SoundPlayingRobotMonitor());
        return result;
    }

    protected void onStart() {
        super.onStart();
        RobotLog.vv("RCActivity", "onStart()");
        this.entireScreenLayout.setOnTouchListener(new 3(this));
    }

    protected void onResume() {
        super.onResume();
        RobotLog.vv("RCActivity", "onResume()");
        ClockWarningSource.getInstance().onPossibleRcClockUpdate();
    }

    protected void onPause() {
        super.onPause();
        RobotLog.vv("RCActivity", "onPause()");
    }

    protected void onStop() {
        super.onStop();
        RobotLog.vv("RCActivity", "onStop()");
    }

    protected void onDestroy() {
        super.onDestroy();
        RobotLog.vv("RCActivity", "onDestroy()");
        this.shutdownRobot();
        if (this.callback != null) {
            this.callback.close();
        }

        PreferenceRemoterRC.getInstance().stop(this.prefRemoterStartResult);
        DeviceNameManagerFactory.getInstance().stop(this.deviceNameStartResult);
        this.unbindFromService();
        ServiceController.stopService(FtcRobotControllerWatchdogService.class);
        if (this.wifiLock != null) {
            this.wifiLock.release();
        }

        if (this.preferencesHelper != null) {
            this.preferencesHelper.getSharedPreferences().unregisterOnSharedPreferenceChangeListener(this.sharedPreferencesListener);
        }

        RobotLog.cancelWriteLogcatToDisk();
        AnnotatedHooksClassFilter.getInstance().callOnDestroyMethods(this);
    }

    protected void bindToService() {
        this.readNetworkType();
        Intent intent = new Intent(this, FtcRobotControllerService.class);
        intent.putExtra("NETWORK_CONNECTION_TYPE", this.networkType);
        this.serviceShouldUnbind = this.bindService(intent, this.connection, 1);
    }

    protected void unbindFromService() {
        if (this.serviceShouldUnbind) {
            this.unbindService(this.connection);
            this.serviceShouldUnbind = false;
        }

    }

    protected void readNetworkType() {
        if (Device.isRevControlHub()) {
            this.networkType = NetworkType.RCWIRELESSAP;
        } else {
            this.networkType = NetworkType.fromString(this.preferencesHelper.readString(this.context.getString(string.pref_pairing_kind), NetworkType.globalDefaultAsString()));
        }

        this.preferencesHelper.writeStringPrefIfDifferent(this.context.getString(string.pref_pairing_kind), this.networkType.toString());
    }

    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) {
            this.immersion.hideSystemUI();
            this.getWindow().setFlags(134217728, 134217728);
        }

    }

    public boolean onCreateOptionsMenu(Menu menu) {
        this.getMenuInflater().inflate(menu.ftc_robot_controller, menu);
        AnnotatedHooksClassFilter.getInstance().callOnCreateMenuMethods(this, menu);
        return true;
    }

    private boolean isRobotRunning() {
        if (this.controllerService == null) {
            return false;
        } else {
            Robot robot = this.controllerService.getRobot();
            if (robot != null && robot.eventLoopManager != null) {
                RobotState robotState = robot.eventLoopManager.state;
                return robotState == RobotState.RUNNING;
            } else {
                return false;
            }
        }
    }

    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();
        Intent intent;
        if (id == id.action_program_and_manage) {
            if (this.isRobotRunning()) {
                intent = new Intent(AppUtil.getDefContext(), ProgramAndManageActivity.class);
                RobotControllerWebInfo webInfo = this.programmingModeManager.getWebServer().getConnectionInformation();
                intent.putExtra("RC_WEB_INFO", webInfo.toJson());
                this.startActivity(intent);
            } else {
                AppUtil.getInstance().showToast(UILocation.ONLY_LOCAL, this.context.getString(string.toastWifiUpBeforeProgrammingMode));
            }
        } else {
            if (id == id.action_inspection_mode) {
                intent = new Intent(AppUtil.getDefContext(), RcInspectionActivity.class);
                this.startActivity(intent);
                return true;
            }

            if (id == id.action_restart_robot) {
                this.dimmer.handleDimTimer();
                AppUtil.getInstance().showToast(UILocation.BOTH, this.context.getString(string.toastRestartingRobot));
                this.requestRobotRestart();
                return true;
            }

            if (id == id.action_configure_robot) {
                EditParameters parameters = new EditParameters();
                Intent intentConfigure = new Intent(AppUtil.getDefContext(), FtcLoadFileActivity.class);
                parameters.putIntent(intentConfigure);
                this.startActivityForResult(intentConfigure, RequestCode.CONFIGURE_ROBOT_CONTROLLER.ordinal());
            } else {
                if (id == id.action_settings) {
                    intent = new Intent(AppUtil.getDefContext(), FtcRobotControllerSettingsActivity.class);
                    this.startActivityForResult(intent, RequestCode.SETTINGS_ROBOT_CONTROLLER.ordinal());
                    return true;
                }

                if (id == id.action_about) {
                    intent = new Intent(AppUtil.getDefContext(), FtcAboutActivity.class);
                    this.startActivity(intent);
                    return true;
                }

                if (id == id.action_exit_app) {
                    this.finishAffinity();
                    if (VERSION.SDK_INT >= 21) {
                        ActivityManager manager = (ActivityManager)this.getSystemService("activity");
                        List<ActivityManager.AppTask> tasks = manager.getAppTasks();
                        Iterator var5 = tasks.iterator();

                        while(var5.hasNext()) {
                            ActivityManager.AppTask task = (ActivityManager.AppTask)var5.next();
                            task.finishAndRemoveTask();
                        }
                    }

                    AppAliveNotifier.getInstance().disableAppWatchdogUntilNextAppStart();
                    AppUtil.getInstance().exitApplication();
                    return true;
                }
            }
        }

        return super.onOptionsItemSelected(item);
    }

    public void onConfigurationChanged(Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
        this.updateMonitorLayout(newConfig);
    }

    private void updateMonitorLayout(Configuration configuration) {
        LinearLayout monitorContainer = (LinearLayout)this.findViewById(id.monitorContainer);
        int i;
        View view;
        if (configuration.orientation == 2) {
            monitorContainer.setOrientation(0);

            for(i = 0; i < monitorContainer.getChildCount(); ++i) {
                view = monitorContainer.getChildAt(i);
                view.setLayoutParams(new LinearLayout.LayoutParams(0, -1, 1.0F));
            }
        } else {
            monitorContainer.setOrientation(1);

            for(i = 0; i < monitorContainer.getChildCount(); ++i) {
                view = monitorContainer.getChildAt(i);
                view.setLayoutParams(new LinearLayout.LayoutParams(-1, 0, 1.0F));
            }
        }

        monitorContainer.requestLayout();
    }

    protected void onActivityResult(int request, int result, Intent intent) {
        if (request == 1 && result == -1) {
            AppUtil.getInstance().showToast(UILocation.BOTH, this.context.getString(string.toastWifiConfigurationComplete));
        }

        if (request == RequestCode.CONFIGURE_ROBOT_CONTROLLER.ordinal() || request == RequestCode.SETTINGS_ROBOT_CONTROLLER.ordinal()) {
            this.shutdownRobot();
            this.cfgFileMgr.getActiveConfigAndUpdateUI();
            this.updateUIAndRequestRobotSetup();
        }

    }

    public void onServiceBind(FtcRobotControllerService service) {
        RobotLog.vv("FTCService", "%s.controllerService=bound", new Object[]{"RCActivity"});
        this.controllerService = service;
        this.updateUI.setControllerService(this.controllerService);
        this.controllerService.setOnBotJavaHelper(this.onBotJavaHelper);
        this.updateUIAndRequestRobotSetup();
        this.programmingModeManager.setState(new 4(this, service));
        AnnotatedHooksClassFilter.getInstance().callWebHandlerRegistrarMethods(this, service.getWebServer().getWebHandlerManager());
    }

    private void updateUIAndRequestRobotSetup() {
        if (this.controllerService != null) {
            this.callback.networkConnectionUpdate(this.controllerService.getNetworkConnectionStatus());
            this.callback.updateRobotStatus(this.controllerService.getRobotStatus());
            this.requestRobotSetup(LynxConstants.isRevControlHub() ? new 5(this) : null);
        }

    }

    private void requestRobotSetup(@Nullable Runnable runOnComplete) {
        if (this.controllerService != null) {
            RobotConfigFile file = this.cfgFileMgr.getActiveConfigAndUpdateUI();
            HardwareFactory hardwareFactory = new HardwareFactory(this.context);

            try {
                hardwareFactory.setXmlPullParser(file.getXml());
            } catch (XmlPullParserException | FileNotFoundException var7) {
                Exception e = var7;
                RobotLog.ww("RCActivity", e, "Unable to set configuration file %s. Falling back on noConfig.", new Object[]{file.getName()});
                file = RobotConfigFile.noConfig(this.cfgFileMgr);

                try {
                    hardwareFactory.setXmlPullParser(file.getXml());
                    this.cfgFileMgr.setActiveConfigAndUpdateUI(false, file);
                } catch (XmlPullParserException | FileNotFoundException var6) {
                    Exception e1 = var6;
                    RobotLog.ee("RCActivity", e1, "Failed to fall back on noConfig");
                }
            }

            OpModeRegister userOpModeRegister = this.createOpModeRegister();
            this.eventLoop = new FtcEventLoop(hardwareFactory, userOpModeRegister, this.callback, this);
            FtcEventLoopIdle idleLoop = new FtcEventLoopIdle(hardwareFactory, userOpModeRegister, this.callback, this);
            this.controllerService.setCallback(this.callback);
            this.controllerService.setupRobot(this.eventLoop, idleLoop, runOnComplete);
            this.passReceivedUsbAttachmentsToEventLoop();
            AndroidBoard.showErrorIfUnknownControlHub();
            AnnotatedHooksClassFilter.getInstance().callOnCreateEventLoopMethods(this, this.eventLoop);
        }
    }

    protected OpModeRegister createOpModeRegister() {
        return new FtcOpModeRegister();
    }

    private void shutdownRobot() {
        if (this.controllerService != null) {
            this.controllerService.shutdownRobot();
        }

    }

    private void requestRobotRestart() {
        AppUtil.getInstance().showToast(UILocation.BOTH, AppUtil.getDefContext().getString(string.toastRestartingRobot));
        RobotLog.clearGlobalErrorMsg();
        RobotLog.clearGlobalWarningMsg();
        this.shutdownRobot();
        this.requestRobotSetup(new 6(this));
    }

    private void showRestartRobotCompleteToast(@StringRes int resid) {
        AppUtil.getInstance().showToast(UILocation.BOTH, AppUtil.getDefContext().getString(resid));
    }

    private void checkPreferredChannel() {
        if (this.networkType == NetworkType.WIFIDIRECT) {
            int prefChannel = this.preferencesHelper.readInt(this.getString(com.qualcomm.ftccommon.R.string.pref_wifip2p_channel), -1);
            if (prefChannel == -1) {
                prefChannel = 0;
                RobotLog.vv("RCActivity", "pref_wifip2p_channel: No preferred channel defined. Will use a default value of %d", new Object[]{prefChannel});
            } else {
                RobotLog.vv("RCActivity", "pref_wifip2p_channel: Found existing preferred channel (%d).", new Object[]{prefChannel});
            }

            RobotLog.vv("RCActivity", "pref_wifip2p_channel: attempting to set preferred channel...");
            this.wifiDirectChannelChanger = new WifiDirectChannelChanger();
            this.wifiDirectChannelChanger.changeToChannel(prefChannel);
        }

    }

    protected void hittingMenuButtonBrightensScreen() {
        ActionBar actionBar = this.getActionBar();
        if (actionBar != null) {
            actionBar.addOnMenuVisibilityListener(new 7(this));
        }

    }

    protected void initWifiMute(boolean enable) {
        if (enable) {
            this.wifiMuteStateMachine = new WifiMuteStateMachine();
            this.wifiMuteStateMachine.initialize();
            this.wifiMuteStateMachine.start();
            this.motionDetection = new MotionDetection(2.0, 10);
            this.motionDetection.startListening();
            this.motionDetection.registerListener(new 8(this));
        } else {
            this.wifiMuteStateMachine.stop();
            this.wifiMuteStateMachine = null;
            this.motionDetection.stopListening();
            this.motionDetection.purgeListeners();
            this.motionDetection = null;
        }

    }

    public void onUserInteraction() {
        if (this.wifiMuteStateMachine != null) {
            this.wifiMuteStateMachine.consumeEvent(WifiMuteEvent.USER_ACTIVITY);
        }

    }
}
