# WebARonARCore Documentation

## Contents

+ [Overview](#Overview)
+ [Disclaimer](#Disclaimer)
+ [Supported devices](#SupportedDevices)
+ [Building/Installing](#BuildingInstalling)
+ [Running examples](#RunningExamples)
+ [Building your own scenes](#BuildingScenes)
+ [Using the AR JavaScript API](#ARJavascriptAPI)
+ [How does WebARonARCore work?](#HowWebARonARCoreWorks)
+ [Known issues](#KnownIssues)
+ [Future work](#FutureWork)
+ [License](#License)


## <a name="Overview">Overview</a>
WebARonARCore is an experimental browser for Android that lets developers create Augmented Reality (AR) experiences for the web, using Android ARCore and JavaScript APIs. It exposes a non standard extension to the WebVR API. The WebARonARCore source/repo includes basic JavaScript examples that developers can use as starting points for their own AR experiences.

The goal of the WebARonARCore project is to enable web developers to create AR experiences on top of Android ARCore using JavaScript.

WebARonARCore can be installed from .APK (which can be built from source). See [instructions](#BuildingInstalling).

## <a name="Disclaimer">Disclaimer</a>
<span style="color:red">**This is not an official Google product.**</span>

<span style="color:red">**The added browser APIs are not on a standards track and are only for experimentation.**</span>

Defining new web APIs is a complex process. The code and ideas in this project are not meant to be definitive proposals for AR capabilities for the web, but prototypes that developers can experiment with, at their own risk.

## <a name="SupportedDevices">Supported devices</a>
WebARonARCore is built on top of Android [ARCore](https://arcore.google.com), which requires a ARCore-compatible Android device. For best results, we recommend one of the following:

* Pixel 2016
* Samsung s8

## <a name="BuildingInstalling">Building/Installing ChromiumARCore</a>
This repository can be used in 2 ways:
[Compile from source](#CompileFromSource)
[Install from prebuilt .apk file](#InstallingAPK)

### <a name="CompileFromSource">Compile from Source</a>
#### Clone the git repo and prepare it to be built

Chromium cloning/building instruction are available to reference online: [https://www.chromium.org/developers/how-tos/android-build-instructions](https://www.chromium.org/developers/how-tos/android-build-instructions)

Prerequisites:

* Linux machine
* GIT
* Python

We recommend you follow the following steps.

1. Open a terminal window
2. Install depot_tools. You can follow this [tutorial](https://commondatastorage.googleapis.com/chrome-infra-docs/flat/depot_tools/docs/html/depot_tools_tutorial.html#_setting_up) or simply follow these 2 steps:
  * `git clone https://chromium.googlesource.com/chromium/tools/depot_tools.git`
  * `export PATH=$PATH:/path/to/depot_tools`
3. Create a folder to contain `chromium`: `$ mkdir ~/chromium && cd ~/chromium`
4. Checkout the Chromium repo: `~/chromium$ fetch --nohooks android`. **NOTE**: This process may take a long time (an hour?)
5. Enter the `src` folder: `$ cd src`.
6. Add this git repo as a remote: `git remote add daydream github.com/blahblahblah`
7. Synchronize the dependencies with this command: `~/chromium/src$ gclient sync --disable-syntax-validation`. **NOTE**: This process may take some time too.
8. Create a folder where to make the final product compilation: `~/chromium/src$ mkdir -p out/master` (you will need to create a folder matching the name of your branch if you are on a branch other than master).
9. Create and edit a new file `out/master/args.gn`. Copy and paste the following content in the `args.gn` file:
```
  target_os = "android"
  target_cpu = "arm64" # or "arm"
  is_debug = false
  # is_official_build = true
  is_component_build = true
  # is_clang = true
  # symbol_level = 1  # Faster build with fewer symbols. -g1 rather than -g2
  # enable_incremental_javac = true
  enable_webvr = true
  proprietary_codecs = false
  ffmpeg_branding = "Chromium"
  enable_nacl = false
  remove_webcore_debug_symbols = true
```
10. Prepare to build: `~/chromium/src$ gn args out/master`. **NOTE**: once the command is executed, the vi editor will show you the content of the `args.gn` file just edited a few steps before. Just exit by pressing ESC and typing colon and `q` with an exclamation mark = `:q!`.
11. Install the build dependencies: `~/chromium/src$ build/install-build-deps-android.sh`
12. Synchronize the resources once again: `~/chromium/src$ gclient sync --disable-syntax-validation`
13. Setup the environment: `~/chromium/src$ . build/android/envsetup.sh`

#### 2. Build, install and run

**IMPORTANT:** Chromium's command buffer has to be modified to enable camera feed rendering. These changes require the command buffer to be rebuilt. The Python script to do so does not execute along with the regular building process so the script needs to be executed with the following command at least once (and everytime any change is made to the command buffer):
```
~/chromium/src$ python gpu/command_buffer/build_gles2_cmd_buffer.py
```
The line below not only compiles Chromium but also the Tango native library called `tango_chromium` that handle the Tango SDK calls. Moreover, the `build_install_run.sh` script also installs the final APK on to a connected device and runs it, so it is convenient that you to connect the Tango device via USB before executing it. The project that will be built by default is the Chromium WebView project, the only one that has been modified to provide Tango/WebAR capabilities.
```
~/chromium/src$ ./build_install_run.sh
```
You can review the content of the script to see what it does (it is a fairly simple script) but if you would like to compile the final APK on your own you could do it by executing the following command:
```
~/chromium/src$ ninja -C out/master
```
The final APK will be built in the folder `~/chromium/src/out/master/out/apks`.

### <a name="InstallingAPK">Installing the APK</a>
* Plug your device into a USB slot on your computer
* Verify that ADB can see the device with `$ adb devices`. You should see your device in the output. If not (adb is very flakey), keep unplugging your device and plugging it back in until it shows up.
* Navigate to `~/chromium/src/third_party/tango`
* Install the tango core APK to your device: `$ adb install -r tango_core.apk`
* Navigate to `~/chromium/src/out/master/out/apks`
* Install the WebARonARCore APK to your device: `$ adb install -r WebARonARCore.apk`
* Launch the WebARonARCore app from your device.

## <a name="RunningExamples">Running examples</a>
To check out examples, open the WebARonARCore app and navigate to [developers.google.com/ar/develop/web/getting-started#examples](https://developers.google.com/ar/develop/web/getting-started#examples) and select an example to see AR on the web.

## <a name="BuildingScenes">Building your own scenes</a>
Check out [developer.google.com/ar/develop/web](https://developers.google.com/ar/develop/web/getting-started) to learn more about how to create your own scenes.

## <a name="ARJavascriptAPI">Using the AR JavaScript API</a>
Documentation for the WebVR extension API for smart phone AR is [here](https://github.com/googlevr/WebARonARCore/blob/master/webvr_ar_extension.md).

## <a name="HowWebARonARCoreWorks">How does WebARonARCore work?</a>
WebARonARCore is made of two essential technologies: ARCore and Chromium.

## <a name="KnownIssues">Known issues</a>
* Because of the nature of how WebARonARCore is built (a webview executing the web content on top of a native process handling ARCore code and the communication between them), it is very hard to get a correct pose estimation that completely matches the underlying camera feed.
* The current implementation of WebAR is built on top of the Chromium WebView flavor. This has some implementation advantages but some performance and use disadvantages. We are working on making the implementation on a full version of Chromium.
* Pausing/resuming/switching away from the app causes screen to turn black. This is a consequence of having built the implementation on top of the WebView flavor of Chromium. A proper implementation on full Chromium or a rebase to a different Chromium WebView version might solve this problem.

## <a name="FutureWork">Future work</a>
* Add more AR-related features.
* Adapt the implementation to the WebVR spec proposal version 2.0.
* Implement the prototype on full Chromium (not on the WebView flavor) and to a newer tag version (>57.0.2987.5).
* Improve the VRPassThroughCamera rendering pipeline either making it obscure for the developer or by using regular WebGL textures and shader samplers without having to use the external image texture extension.
* Add more Tango SDK "hidden" features: mesh reconstruction, ...

## <a name="License">License</a>
Apache License Version 2.0 (see the `LICENSE' file inside this repo).

