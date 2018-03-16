# Install Guide
This is the Parafoil/PlatformIO install guide! You should be able to (1) build code onto Teensies after this tutorial, (2) pull/push to git with a few clicks, and (3) edit code with others like Google Docs. *This guide is for installing on the Atom editor (recommended)*


1 - You should already have this repo cloned to your computer, which you can do with GIT commands, or by going [here](https://github.com/stanford-ssi/balloons-Parafoil) and clicking "Clone or Download" > Download ZIP.

2 - **Mac OS X:* Open Terminal and run "clang --version". It should tell you your LLVM version if properly installed. Otherwise, install XCode Command Line Tools.
    **Windows:** Download the Clang 3.9.1 [32-bit](http://releases.llvm.org/3.9.1/LLVM-3.9.1-win32.exe) or [64-bit](http://releases.llvm.org/3.9.1/LLVM-3.9.1-win64.exe).
    ##*Important Note: Windows users can select default options during the install, but make sure to click "Add LLVM to the system PATH" when prompted.* **This is not the default option.**

3 - Download and install
 - [this](https://git-scm.com/downloads) git-related thing. --> It enables github integration
 - the [Atom Editor](https://atom.io/)
 - the [Teensie Loader App](https://www.pjrc.com/teensy/loader.html)

4 - Navigate to settings *Control/Command + Comma*

5 - Click on the Install tab (it has a + icon)

6 - Search for and install the following packages **in this order**
 - teletype
 - "platformio-ide"

7 - Click "Open Project" and navigate to "balloons-parafoil/parafoil-avionics/parafoil-avionics" and click "Select Folder". *This directory hierarchy is definitely subject to change*

8 - Double click "src/main.cpp"

9 - You should now see
 - Up and down arrows (in that order) in the bottom right hand corner.
 - A little signal tower icon in the bottom right hand corner

10 - You should not see
 - A message along the lines of Note... origin... something... when you click the up and down arrows in the bottom right hand corner
 - Any error messages tbh

11 - If you run into any problems, whine at @bt on Slack or Google it (either way Slack me so I can update the guide)

12 - If everything worked, you should be able to
 - Compile (the check mark in the top left corner, or alt+command/control+b)
 - Pull changes from git by clicking the up/down arrows in the bottom left corner, clicking "Fetch", and then "Pull"
 - Share your session by clicking on the signal tower icon in the bottom left corner and following the directions (Login through Github and share your session code)

 FAQ
 - If PlatformIO couldn't find a library, click "PlatformIO>Rebuild C/C++ Project Index (Autocomplete, Linter)"
 - Stage and commit changes before pushing by clicking the file counter in the bottom right corner (it should say "[#] files"), adding relevant files, adding a commit message, and clicking "Commit". Push by clicking the arrows and clicking the up arrow when the menu comes up. (Make sure to pull first!)
 - If opening the terminal in PlatformIO hangs, go to the packages menu and reinstall "platformio-ide-terminal"

### *Last Updated: 3/15/18 @ 10:14PM
